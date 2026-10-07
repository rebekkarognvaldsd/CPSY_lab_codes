#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cinttypes>
#include <signal.h>
#include <oneapi/tbb.h>
#include <lgpio.h>

/* The configuration is currently hard coded. Redefine these macros to the values
 * You need or implement a configuration file mechanism. */
#define GPIO_A1 27
#define GPIO_A2 17
#define GPIO_B1 6
#define GPIO_B2 5
#define RATE  50 /* Once every 20ms */
#define ITERS 50
// We expect about 2.5 revolutions per minute, or 2.5 * 700*4 pin changes. We add a few more for slack.
#define STEPS (3 * 700 * 4 / RATE)


/*
 * We describe the quadrature decoder by a Mealy machine. `state_t` defines the states,
 * `transitions` defines the transition matrix (first index is a state and the second one the symbol).
 * The function output defines the output symbol (-1, 0, and 1) which we use to increment the step counter.
 */
typedef enum : uint_fast8_t {
    QDECODER_STATE_IDLE,
    QDECODER_STATE_CCW1,
    QDECODER_STATE_CCW2,
    QDECODER_STATE_CCW3,
    QDECODER_STATE_CW1,
    QDECODER_STATE_CW2,
    QDECODER_STATE_CW3,
} state_t;

const state_t transitions[7][4] = {
    { QDECODER_STATE_IDLE, QDECODER_STATE_CCW1, QDECODER_STATE_CW1, QDECODER_STATE_IDLE },  // IDLE
    { QDECODER_STATE_IDLE, QDECODER_STATE_CCW1, QDECODER_STATE_CCW3, QDECODER_STATE_CCW2 }, // CCW1
    { QDECODER_STATE_IDLE, QDECODER_STATE_CCW1, QDECODER_STATE_CCW3, QDECODER_STATE_CCW2 }, // CCW2
    { QDECODER_STATE_IDLE, QDECODER_STATE_IDLE, QDECODER_STATE_CCW3, QDECODER_STATE_CCW2 }, // CCW3
    { QDECODER_STATE_IDLE, QDECODER_STATE_CW3, QDECODER_STATE_CW1, QDECODER_STATE_CW2 },    // CW1
    { QDECODER_STATE_IDLE, QDECODER_STATE_CW3, QDECODER_STATE_CW1, QDECODER_STATE_CW2 },    // CW2
    { QDECODER_STATE_IDLE, QDECODER_STATE_CW3, QDECODER_STATE_IDLE, QDECODER_STATE_CW2 },   // CW3
};


/* We describe the output of the transition as a function instead of a table. */
inline int output(const state_t from, const uint_fast8_t symbol)
{
    if (QDECODER_STATE_CCW3 == from && 0 == symbol)
        return -1;
    if (QDECODER_STATE_CW3 == from && 0 == symbol)
        return 1;
    return 0;
}


/* To order events of type `lgGpioReport_t` by their timestamp. */
bool operator<(const lgGpioReport_t &first, const lgGpioReport_t &second)
{
    return first.timestamp > second.timestamp;
}


class encoder_state {
public:
    uint_fast8_t last_symbol;
    state_t qdecoder_state;
    uint8_t pin1, pin2;
    int64_t steps;
    uint64_t timestamp;
    oneapi::tbb::concurrent_priority_queue<lgGpioReport_t> inputs;

    // We do not want to have the copy constructor, copy assignment constructor, and move assignment operator.
    encoder_state() = delete;
    encoder_state(uint8_t _pin1, uint8_t _pin2);
    ~encoder_state() = default;
    encoder_state(encoder_state&) = delete;
    encoder_state& operator=(const encoder_state&) = delete;
    encoder_state& operator=(encoder_state&&) = delete;

    uint_fast8_t symbol(const lgGpioReport_t&) const noexcept;
    void step(const lgGpioReport_t&) noexcept;
    void run(const int steps = STEPS) noexcept;
};


encoder_state::encoder_state(uint8_t _pin1, uint8_t _pin2) : pin1(_pin1), pin2(_pin2)
{
    // Empty
}

/* Obtain the symbol from an event and the current state.
 *
 * Each event records the change of a single pin, so we change the last known symbol,
 * a pair of pin values, to calculate the next pin.
 */
uint_fast8_t
encoder_state::symbol(const lgGpioReport_t &report) const noexcept
{
    assert((pin1 == report.gpio) || (pin2 == report.gpio));
    if (report.gpio == pin1)
        return (last_symbol & ~1) | static_cast<uint_fast8_t>(report.level);
    return (last_symbol & ~2) | (static_cast<uint_fast8_t>(report.level) << 1);
}


/* Ubdate the state of the quadrature decoder machine. */
inline void
encoder_state::step(const lgGpioReport_t &event) noexcept
{
#if 0
    if (event.timestamp < timestamp) {
    const double delta = static_cast<double>(timestamp - event.timestamp) /* ns */ / 1000000.0;
        printf("Event processed %fms late\n", delta);
    }
#endif
    state_t from = qdecoder_state;
    last_symbol = symbol(event);
    timestamp = event.timestamp;
    qdecoder_state = transitions[from][last_symbol];
    steps += output(from, last_symbol);
    //printf("   [%d ---%d/%d---> %d] steps=%" PRId64 "\n", state.qdecoder_state, state.symbol, output(from, state.qdecoder_state), state.qdecoder_state, state.steps);
}


/* Run the state machine for `steps` steps, or as long as the input queue is empty.
 * Choose the number of steps to empty the queue in most of the situations.
 */
void
encoder_state::run(int steps) noexcept
{
    lgGpioReport_t event;
    for (int i = 0; i < STEPS; i++) {
        if (!inputs.try_pop(event))
            return;
        step(event);
    }
}

/* Callback function.
 * When the value of a pin changes, this function is called to enqueue the event into our
 * priotrity queue.
 *
 * This function is thread-safe.
 */
void afunc(int e, lgGpioAlert_p evt, void *data)
{
    encoder_state *state = static_cast<encoder_state *>(data);

    for (int i=0; i<e; i++) {
        const lgGpioReport_t &report = evt[i].report;
        if (report.flags != 0) {
            continue;
        }
        state->inputs.push(report);
    }
}

/* Global variables. */
int chip;
encoder_state left(GPIO_B1, GPIO_B2), right(GPIO_A1, GPIO_A2);

/* When we exit. */
void exit_handler()
{
    int status = lgGpiochipClose(chip);
    if (status < 0) {
        printf("Failed to close gpiochip %d\n", chip);
    }
    printf("left = %" PRId64 " right = %" PRId64 "\n", left.steps, right.steps);
}

void sigterm_handler(int signum)
{
    printf("Received SIGTERM.\n");
    exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
    if (argc == 5) {
        left.pin1 = atoi(argv[1]);
        left.pin2 = atoi(argv[2]);
        right.pin1 = atoi(argv[3]);
        right.pin2 = atoi(argv[4]);
    } else if (argc != 1) {
        exit(EXIT_FAILURE);
    }

    struct sigaction sa;
    sa.sa_handler = sigterm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }

    chip = lgGpiochipOpen(0);
    if (chip < 0) {
        // TODO: Handle error
        exit(EXIT_FAILURE);
    }
    atexit(exit_handler);

    lgGpioSetAlertsFunc(chip, left.pin1, afunc, &left);
    lgGpioSetAlertsFunc(chip, left.pin2, afunc, &left);
    lgGpioSetAlertsFunc(chip, right.pin1, afunc, &right);
    lgGpioSetAlertsFunc(chip, right.pin2, afunc, &right);
    lgGpioClaimAlert(chip, LG_SET_PULL_UP, LG_BOTH_EDGES, left.pin1, -1);
    lgGpioClaimAlert(chip, LG_SET_PULL_UP, LG_BOTH_EDGES, left.pin2, -1);
    lgGpioClaimAlert(chip, LG_SET_PULL_UP, LG_BOTH_EDGES, right.pin1, -1);
    lgGpioClaimAlert(chip, LG_SET_PULL_UP, LG_BOTH_EDGES, right.pin2, -1);

    for (;;) {
        for (int i = 0; i < ITERS; i++) {
            lguSleep(1.0/static_cast<double>(RATE));
            left.run();
            right.run();
            printf("%" PRId64 " %" PRId64 "\n", left.steps, right.steps);
            fflush(stdout);
        }

    }
}