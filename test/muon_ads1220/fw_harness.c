// Run the real src/sensor_muon_ads1220.c on the host (KAN-4835)
//
// The firmware source is included whole, so its step() state machine, its
// commands and its blanking are what run here. Around it sit a model of
// the ADS1220 behind spidev_transfer(), a model of the probe trigger
// (trigger_analog_*), a bulk stream that counts samples, and a clock.
//
// The chip model is the one fact the test cannot take from the datasheet:
// after the mux returns to the load cell, the load cell channel carries a
// step that decays over a few ms (GLITCH_*). The bench false trigger of
// 19 Nov (KAN-4835) is that step reaching the trigger, so every scenario
// that expects no false trigger is also run with the handover left off,
// where the same model must false trigger (the negative control).
//
// Usage: fw_harness [off]    exits non-zero on a failed check

#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/command.h"

// Replace the message and shutdown macros before the firmware sees them
#undef sendf
#undef shutdown
#define sendf(FMT, args...) harness_sendf(FMT, ##args)
#define shutdown(MSG) harness_shutdown(MSG)
static void harness_sendf(const char *fmt, ...);
static void harness_shutdown(const char *msg);

#include "../../src/sensor_muon_ads1220.c"

#define HZ 12000000.
#define US(x) ((uint32_t)((x) * 1e-6 * HZ))
#define SEC(x) ((uint32_t)((x) * HZ))

// Chip timing, as klippy/extras/muon_ads1220.py derives it at 2000SPS
#define T_CLK (1. / 4096000.)
#define CONV_S (2072 * T_CLK)
#define FIRST_S ((114. + 2130.) * T_CLK)
#define OSC_TOL 0.03
#define MARGIN_S 20e-6

// The model
#define LC_BASE 1000000     // load cell counts at rest
#define TEMP_COUNTS 3000000 // what the hotend channel reads
#define TRIG 100000         // trigger: |counts - tare| >= TRIG
#define GLITCH_COUNTS 400000.
#define GLITCH_TAU_S 3e-3
#define SPEED_MM_S 15.

static uint32_t now;
static int jmp_valid;
static jmp_buf jmp;
static char shutdown_msg[128];

// Scenario knobs
static int glitch_on = 1;
static double osc_err = 0.01;
static uint32_t contact_tick;       // absolute time of the load step
static double contact_step = 0.;
static double settle_blank_s = 0.010, max_blank_s = 0.050;

// Chip state
static struct {
    uint8_t regs[4];
    uint32_t restart;        // time of the last register 0 write
    uint8_t lc;              // reg0 selects the load cell
    uint32_t lc_since;       // when the mux last went to the load cell
    uint8_t lc_reg0;
} chip;

// Trigger model
static struct trigger_analog {
    int armed, fired, error, alive;
    int32_t tare;
    uint32_t fire_t, arm_t;
    int fed;
    int32_t worst;           // largest |counts - tare| fed while armed
} ta;

// Bulk stream counters
static uint32_t n_real, n_filler;

// Messages from the mcu
static struct {
    int blanked;
    uint32_t b_start, b_end, b_dropped;
    int temps;
} msgs;

static void
harness_sendf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    if (!strncmp(fmt, "muon_ads1220_blanked", 20)) {
        (void)va_arg(ap, unsigned);
        msgs.b_start = va_arg(ap, unsigned);
        msgs.b_end = va_arg(ap, unsigned);
        msgs.b_dropped = va_arg(ap, unsigned);
        msgs.blanked++;
    } else if (!strncmp(fmt, "muon_ads1220_temp", 17)) {
        msgs.temps++;
    }
    va_end(ap);
}

static void
harness_shutdown(const char *msg)
{
    snprintf(shutdown_msg, sizeof(shutdown_msg), "%s", msg);
    if (jmp_valid)
        longjmp(jmp, 1);
    fprintf(stderr, "unexpected shutdown: %s\n", msg);
    exit(2);
}

/****************************************************************
 * Stubs for the rest of the mcu
 ****************************************************************/

uint32_t timer_read_time(void) { return now++; }
void irq_disable(void) { }
void irq_enable(void) { }
struct gpio_out gpio_out_setup(uint32_t pin, uint8_t val)
{
    struct gpio_out g = { (int)pin };
    return g;
}
void gpio_out_write(struct gpio_out g, uint8_t val) { }

static struct muon_ads1220 chip_obj;
void *oid_alloc(uint8_t oid, void *type, uint16_t size)
{
    memset(&chip_obj, 0, sizeof(chip_obj));
    return &chip_obj;
}
void *oid_lookup(uint8_t oid, void *type) { return &chip_obj; }
void *oid_next(uint8_t *i, void *type) { return NULL; }
void sched_add_timer(struct timer *t) { }
void sched_del_timer(struct timer *t) { }
void sched_wake_task(struct task_wake *w) { w->wake = 1; }
uint8_t sched_check_wake(struct task_wake *w) { return w->wake; }
struct spidev_s *spidev_oid_lookup(uint8_t oid) { return NULL; }

void sensor_bulk_reset(struct sensor_bulk *sb)
{
    sb->data_count = 0;
    sb->possible_overflows = 0;
}
void sensor_bulk_report(struct sensor_bulk *sb, uint8_t oid)
{
    for (int i = 0; i + 4 <= sb->data_count; i += 4) {
        int32_t v = sb->data[i] | (sb->data[i+1] << 8) | (sb->data[i+2] << 16)
                    | ((uint32_t)sb->data[i+3] << 24);
        if (v == MADS_FILLER)
            n_filler++;
        else
            n_real++;
    }
    sb->data_count = 0;
}
void sensor_bulk_status(struct sensor_bulk *sb, uint8_t oid, uint32_t t1
                        , uint32_t q, uint32_t fifo) { }

struct trigger_analog *trigger_analog_oid_lookup(uint8_t oid) { return &ta; }
int trigger_analog_is_armed(struct trigger_analog *t)
{
    return t && t->armed;
}
void trigger_analog_note_alive(struct trigger_analog *t) { t->alive++; }
void trigger_analog_note_error(struct trigger_analog *t, uint8_t code)
{
    if (!t->armed)
        return;
    t->armed = 0;
    t->error = code + 1;
}
void trigger_analog_update(struct trigger_analog *t, int32_t sample)
{
    if (!t->armed)
        return;
    t->fed++;
    int32_t d = abs(sample - t->tare);
    if (d > t->worst)
        t->worst = d;
    if (d >= TRIG) {
        t->fired = 1;
        t->fire_t = now;
        t->armed = 0;
    }
}

/****************************************************************
 * The chip
 ****************************************************************/

static uint32_t
conv_ticks(void) { return (uint32_t)(CONV_S * (1. + osc_err) * HZ); }
static uint32_t
first_ticks(void) { return (uint32_t)(FIRST_S * (1. + osc_err) * HZ); }

static uint32_t rng = 12345;
static double
noise(void)
{
    rng = rng * 1664525u + 1013904223u;
    return ((rng >> 8) & 0xffff) / 65535. - .5;
}

static int32_t
channel_value(uint32_t t)
{
    if (!chip.lc)
        return TEMP_COUNTS + (int32_t)(noise() * 200);
    double v = LC_BASE + noise() * 600.;
    double since = (int32_t)(t - chip.lc_since) / HZ;
    if (glitch_on && since >= 0)
        v += GLITCH_COUNTS * exp(-since / GLITCH_TAU_S);
    if (contact_tick && (int32_t)(t - contact_tick) >= 0)
        v += contact_step;
    return (int32_t)v;
}

static void
set_reg0(uint8_t reg0)
{
    int was_lc = chip.lc;
    chip.regs[0] = reg0;
    chip.lc = (reg0 == chip.lc_reg0);
    chip.restart = now;
    if (chip.lc && !was_lc)
        chip.lc_since = now;
}

void spidev_transfer(struct spidev_s *spi, uint8_t receive, uint8_t len
                     , uint8_t *d)
{
    now += US(8);               // an SPI transfer takes about this long
    uint8_t cmd = d[0] & 0xf0;
    if (len == 1) {
        if (d[0] == CMD_RESET)
            memset(chip.regs, 0, sizeof(chip.regs));
        return;
    }
    if (cmd == CMD_WREG && len == 2) {
        set_reg0(d[1]);
    } else if (cmd == CMD_WREG) {
        for (int i = 0; i < 4; i++)
            chip.regs[i] = d[1 + i];
        chip.lc = (chip.regs[0] == chip.lc_reg0);
        chip.restart = now;
    } else if (cmd == CMD_RREG) {
        for (int i = 0; i < 4; i++)
            d[1 + i] = chip.regs[i];
    } else if (d[0] == CMD_RDATA) {
        // The latest conversion that has completed
        uint32_t c = conv_ticks(), f = first_ticks();
        int32_t since = (int32_t)(now - chip.restart);
        uint32_t t = chip.restart;
        if (since >= (int32_t)f)
            t = chip.restart + f + ((since - f) / c) * c;
        int32_t v = channel_value(t);
        d[1] = v >> 16;
        d[2] = v >> 8;
        d[3] = v;
    }
}

/****************************************************************
 * Running the firmware
 ****************************************************************/

#define OID 0

static uint32_t read_ticks_v;

static void
setup(int handover)
{
    memset(&chip, 0, sizeof(chip));
    memset(&ta, 0, sizeof(ta));
    memset(&msgs, 0, sizeof(msgs));
    n_real = n_filler = 0;
    now = 1000;
    jmp_valid = 0;
    shutdown_msg[0] = 0;
    uint32_t args[16];
    args[0] = OID; args[1] = 0; args[2] = 5; args[3] = 0;
    command_config_muon_ads1220(args);
    struct muon_ads1220 *a = &chip_obj;
    // klippy/extras/muon_ads1220.py: lc_reg0 mux AIN0_AIN1 gain 64
    uint8_t lc_reg0 = (0 << 4) | (6 << 1);
    chip.lc_reg0 = lc_reg0;
    args[0] = OID; args[1] = lc_reg0; args[2] = 0x74;
    args[3] = 0x40; args[4] = 0; args[5] = 0x2a; args[6] = 0x3a;
    args[7] = 0x40; args[8] = 0; args[9] = 0x31;
    command_muon_ads1220_set_regs(args);
    double conv = CONV_S, first = FIRST_S;
    read_ticks_v = (uint32_t)(conv / 2.2 * HZ);
    args[0] = OID; args[1] = read_ticks_v; args[2] = (uint32_t)(conv * HZ);
    args[3] = (uint32_t)(conv * (1. - OSC_TOL) * HZ);
    args[4] = (uint32_t)(conv * (1. + OSC_TOL) * HZ);
    command_muon_ads1220_set_timing(args);
    args[0] = OID;
    args[1] = SEC((first + conv) * (1. + OSC_TOL) + MARGIN_S);
    args[2] = SEC(first);
    args[3] = SEC(first * (1. + OSC_TOL) + MARGIN_S);
    args[4] = SEC(0.200);
    args[5] = SEC(4.0);
    args[6] = 64;
    command_muon_ads1220_set_schedule(args);
    args[0] = OID; args[1] = 0;
    command_muon_ads1220_attach_trigger_analog(args);
    if (handover) {
        args[0] = OID;
        args[1] = SEC(settle_blank_s);
        args[2] = SEC(max_blank_s);
        command_muon_ads1220_set_handover(args);
    }
    args[0] = OID; args[1] = MODE_PT1000;
    chip.regs[0] = a->sense_reg0[MODE_PT1000];
    command_muon_ads1220_start(args);
    args[0] = OID; args[1] = 1;
    command_muon_ads1220_lc_enable(args);
}

// Run the task until time 'until' (seconds since start), calling 'hook' at
// every step. 'skip_until' (a time) models a task that is not run.
static uint32_t skip_until;
static void
run(double until, void (*hook)(double))
{
    struct muon_ads1220 *a = &chip_obj;
    uint32_t t0 = now;
    while ((now - t0) < SEC(until)) {
        now += read_ticks_v;
        double t = (now - t0) / HZ;
        if (hook)
            hook(t);
        if (skip_until && (int32_t)(now - skip_until) < 0)
            continue;
        if (a->flags & MF_RUNNING)
            step(a, OID);
    }
}

static void
arm(void)
{
    ta.armed = 1;
    ta.fired = ta.error = 0;
    ta.tare = LC_BASE;
    ta.worst = 0;
    ta.fed = 0;
    ta.arm_t = now;
}

static double arm_after_s;
static void
arm_hook(double t)
{
    if (!ta.armed && !ta.fired && !ta.error && t >= arm_after_s)
        arm();
}

/****************************************************************
 * Checks
 ****************************************************************/

static int failures, expect_off;
#define CHECK(cond, ...) do { \
        if (cond) { printf("ok   "); } else { printf("FAIL "); failures++; } \
        printf(__VA_ARGS__); printf("\n"); } while (0)

static void
reset_knobs(void)
{
    glitch_on = 1;
    osc_err = 0.01;
    contact_tick = 0;
    contact_step = 0.;
    settle_blank_s = 0.010;
    max_blank_s = 0.050;
    skip_until = 0;
}

// 1. A probe armed for longer than max_temperature_pause: the read is taken
//    mid-probe and must not fire the trigger.
static void
scenario_midair(int handover)
{
    reset_knobs();
    setup(handover);
    arm_after_s = 0.3;
    run(5.0, arm_hook);
    struct muon_ads1220 *a = &chip_obj;
    double ms = a->blank.max_window / HZ * 1e3;
    if (handover) {
        CHECK(!ta.fired && !ta.error, "mid-air read with the handover: no"
              " false trigger (worst fed %d counts of %d)", ta.worst, TRIG);
        CHECK(msgs.blanked >= 1, "the blanked window is reported (%d)",
              msgs.blanked);
        CHECK(ms > 0. && ms <= max_blank_s * 1e3,
              "window %.2f ms is within the %.0f ms hard cap", ms,
              max_blank_s * 1e3);
        CHECK(n_filler > 0 && a->blank.n_dropped > 0,
              "blanked samples left the stream as fillers (%u dropped)",
              a->blank.n_dropped);
        CHECK(ta.alive > 0, "the trigger's liveness monitor is fed");
        printf("     window %.2f ms = %.3f mm at %.0f mm/s\n", ms,
               ms * 1e-3 * SPEED_MM_S, SPEED_MM_S);
    } else {
        CHECK(ta.fired, "negative control: without the handover the same"
              " model false triggers (worst %d counts)", ta.worst);
    }
}

// 2. A real contact during the blind window must be caught right after it.
static double contact_into_us;
static void
contact_hook(double t)
{
    arm_hook(t);
    if (!contact_tick && ta.armed && chip_obj.blank.active)
        contact_tick = chip_obj.blank.t0 + US(contact_into_us);
}

static void
scenario_contact(double into_window_ms)
{
    reset_knobs();
    setup(1);
    arm_after_s = 0.3;
    contact_into_us = into_window_ms * 1e3;
    contact_tick = 0;
    contact_step = 300000.;
    run(4.6, contact_hook);
    struct muon_ads1220 *a = &chip_obj;
    double fire_ms = 0, win_ms = a->blank.max_window / HZ * 1e3;
    if (ta.fired)
        fire_ms = (int32_t)(ta.fire_t - contact_tick) / HZ * 1e3;
    CHECK(ta.fired && !ta.error, "contact %.1f ms into the window is caught"
          " (fired %.2f ms after contact)", into_window_ms, fire_ms);
    CHECK(ta.fired && ta.worst >= TRIG, "caught on a real load (%d counts)",
          ta.worst);
    if (ta.fired) {
        double conv_ms = conv_ticks() / HZ * 1e3;
        // No sample is fed until the window closes, then the very next one
        // trips: so the latency is bounded by the window plus a conversion
        CHECK(fire_ms <= win_ms + 2. * conv_ms,
              "latency %.2f ms <= window %.2f ms + 2 conversions",
              fire_ms, win_ms);
        printf("     extra travel %.3f mm at %.0f mm/s\n",
               fire_ms * 1e-3 * SPEED_MM_S, SPEED_MM_S);
    }
}

static double stall_ms;
static int stalled;
static void
stall_hook(double t)
{
    arm_hook(t);
    if (!stalled && ta.armed && chip_obj.blank.active) {
        stalled = 1;
        skip_until = now + US(stall_ms * 1e3);
    }
}

// 3. A window that runs past its hard limit stops the probe (never extends)
static void
scenario_overrun(void)
{
    reset_knobs();
    setup(1);
    stalled = 0;
    arm_after_s = 0.3;
    // the task stops for 80 ms just after the armed slice starts (an
    // interrupt storm): the window would be 80 ms+, past its hard limit
    stall_ms = 80.;
    run(5.0, stall_hook);
    struct muon_ads1220 *a = &chip_obj;
    CHECK(ta.error == ME_BLANK + 1 && !ta.armed,
          "a blind window past %.0f ms stops the probe (error %d)",
          max_blank_s * 1e3, ta.error);
    CHECK(a->blank.n_over == 1, "the overrun is counted (%u)",
          a->blank.n_over);
}

// 4. Not armed: nothing changes (fillers and samples as without the feature)
static void
scenario_unarmed(void)
{
    uint32_t real[2], fill[2];
    for (int h = 0; h < 2; h++) {
        reset_knobs();
        setup(h);
        arm_after_s = 1e9;
        run(2.0, arm_hook);
        real[h] = n_real;
        fill[h] = n_filler;
        CHECK(!chip_obj.blank.n_windows, "unarmed (handover %d): no window"
              " reported", h);
    }
    CHECK(real[0] == real[1] && fill[0] == fill[1], "unarmed: the stream is"
          " identical with and without the handover (%u/%u)", real[1],
          fill[1]);
}

// 5. The firmware refuses a window beyond its own hard limit
static void
scenario_range(void)
{
    reset_knobs();
    setup(0);
    uint32_t args[3] = { OID, SEC(0.010), SEC(0.080) };
    jmp_valid = 1;
    if (!setjmp(jmp))
        command_muon_ads1220_set_handover(args);
    jmp_valid = 0;
    CHECK(shutdown_msg[0] != 0, "a configured window over %d ms is refused"
          " (%s)", BLANK_HARD_MAX_US / 1000, shutdown_msg);
    shutdown_msg[0] = 0;
    args[1] = SEC(0.060);
    args[2] = SEC(0.050);
    jmp_valid = 1;
    if (!setjmp(jmp))
        command_muon_ads1220_set_handover(args);
    jmp_valid = 0;
    CHECK(shutdown_msg[0] != 0, "a settle time longer than the window is"
          " refused");
}

int
main(int argc, char **argv)
{
    expect_off = argc > 1 && !strcmp(argv[1], "off");
    if (expect_off) {
        scenario_midair(0);
    } else {
        scenario_midair(1);
        scenario_midair(0);
        scenario_contact(0.5);
        scenario_contact(3.0);
        scenario_overrun();
        scenario_unarmed();
        scenario_range();
    }
    printf("%d failures\n", failures);
    return failures ? 1 : 0;
}
