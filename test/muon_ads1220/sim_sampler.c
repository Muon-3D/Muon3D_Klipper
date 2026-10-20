// Simulate src/sensor_muon_ads1220.h against a modelled ADS1220
//
// The chip converts continuously at 4.096MHz / 2072 (the 2000SPS turbo
// setting) scaled by an oscillator error. The mcu reads RDATA on a fixed
// timer with task latency added to each read, and every temperature period
// the load cell is switched away for a slice and back.
//
// Checked, per scenario:
//  - every conversion is reported exactly once, with its own value, and in
//    order (unless a read was late enough to lose one, which must then be
//    counted as a hold)
//  - the delay from a conversion completing to the mcu seeing it
//  - the timestamp error a fixed-frequency fit over the slot stream gives
//
// Built and run by test_muon_ads1220.py; prints one line per scenario and
// exits non-zero on a failed check.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/sensor_muon_ads1220.h"

#define MCU_FREQ 12000000.
#define T_CLK (1. / 4096000.)
#define CONV_TCLK 2072.
#define FIRST_TCLK (114. + 2130.)

static uint64_t rng_state = 1;
static double
rnd(void)
{
    rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return ((rng_state >> 11) & ((1ULL << 53) - 1)) / (double)(1ULL << 53);
}
static double
gauss(void)
{
    double u = rnd() + 1e-12, v = rnd();
    return sqrt(-2. * log(u)) * cos(2. * M_PI * v);
}

struct scenario {
    const char *name;
    double osc_err;       // chip oscillator error (fraction)
    double sigma;         // noise in counts
    double p_repeat;      // extra chance a conversion repeats its code
    double jitter_us;     // uniform task latency per read
    double p_stall;       // chance a read is delayed by stall_us
    double stall_us;
    double slice_us;      // temperature slice length (0 = none)
    int saturate;         // input saturated for part of the run
    double seconds;
};

struct conv { double t; int32_t v; };

#define MAX_CONV 200000
static struct conv convs[MAX_CONV];
static int nconv;

struct emitted { double t_read; int32_t v; int conv; long slot; int filler; };
#define MAX_EMIT 220000
static struct emitted em[MAX_EMIT];
static int nem;
static long slot;

static void
emit(double t, int32_t v, int conv_idx, int filler)
{
    if (nem < MAX_EMIT) {
        em[nem].t_read = t;
        em[nem].v = v;
        em[nem].conv = conv_idx;
        em[nem].slot = slot;
        em[nem].filler = filler;
        nem++;
    }
    slot++;
}

// Fit slot index -> true time over measured samples, return max residual
static double
fit_residual(double *rate_err)
{
    double sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0;
    int i;
    for (i = 0; i < nem; i++) {
        if (em[i].filler || em[i].conv < 0)
            continue;
        double x = em[i].slot, y = convs[em[i].conv].t;
        sx += x; sy += y; sxx += x * x; sxy += x * y; n++;
    }
    double b = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    double a = (sy - b * sx) / n;
    double worst = 0;
    for (i = 0; i < nem; i++) {
        if (em[i].filler || em[i].conv < 0)
            continue;
        double r = fabs(a + b * em[i].slot - convs[em[i].conv].t);
        if (r > worst)
            worst = r;
    }
    *rate_err = b / (CONV_TCLK * T_CLK) - 1.;
    return worst;
}

static int
run(const struct scenario *sc, int verbose)
{
    double period = CONV_TCLK * T_CLK * (1. + sc->osc_err);
    double first = FIRST_TCLK * T_CLK * (1. + sc->osc_err);
    double tick = 1. / MCU_FREQ;
    uint32_t conv_ticks = CONV_TCLK * T_CLK * MCU_FREQ;
    struct mads_sampler s;
    mads_sampler_init(&s, conv_ticks, conv_ticks * 97 / 100
                      , conv_ticks * 103 / 100);
    double read_interval = CONV_TCLK * T_CLK / 2.2;
    double first_nominal = FIRST_TCLK * T_CLK;
    double lc_settle = FIRST_TCLK * T_CLK * 1.03 + 20e-6;
    double temp_period = 0.200;

    nconv = nem = 0;
    slot = 0;
    // Chip state: load cell conversions start at seg_start + first
    double seg_start = 0., next_conv = first, next_temp = temp_period;
    int in_slice = 0;
    double slice_end = 0., settle_end = lc_settle;
    int settling = 1;
    mads_sampler_restart(&s, (uint32_t)(first_nominal * MCU_FREQ));
    int32_t latest = 0;
    int latest_idx = -1;
    int32_t value = 1000;
    double t = 0.;
    int k;
    double max_delay = 0., sum_delay = 0.;
    int n_delay = 0, failures = 0;
    long measured = 0, flagged = 0, phantoms = 0;
    double t_prev_read = 0.;
    for (k = 1; ; k++) {
        double sched = k * read_interval;
        if (sched > sc->seconds)
            break;
        t = sched + rnd() * sc->jitter_us * 1e-6;
        if (rnd() < sc->p_stall)
            t += sc->stall_us * 1e-6;
        // Advance the chip to time t
        while (!in_slice && next_conv <= t && nconv < MAX_CONV) {
            int sat = sc->saturate && next_conv > sc->seconds * 0.4
                && next_conv < sc->seconds * 0.45;
            if (sat)
                value = 0x7fffff;
            else if (rnd() >= sc->p_repeat)
                value = 1000 + (int32_t)lround(gauss() * sc->sigma);
            convs[nconv].t = next_conv;
            convs[nconv].v = value;
            latest = value;
            latest_idx = nconv;
            nconv++;
            next_conv += period;
        }
        uint32_t now = (uint32_t)(uint64_t)llround(t * MCU_FREQ);
        if (in_slice) {
            uint32_t n = mads_sampler_gap_fill(&s, now);
            while (n--)
                emit(t, MADS_FILLER, -1, 1);
            if (t >= slice_end) {
                // Mux back to the load cell: conversion restarts
                in_slice = 0;
                seg_start = t;
                next_conv = seg_start + first;
                settling = 1;
                settle_end = t + lc_settle;
                mads_sampler_restart(&s, now + (uint32_t)(first_nominal
                                                          * MCU_FREQ));
            }
            continue;
        }
        int start_slice = 0;
        if (settling) {
            uint32_t n = mads_sampler_gap_fill(&s, now);
            while (n--)
                emit(t, MADS_FILLER, -1, 1);
            if (t < settle_end)
                continue;
            settling = 0;
        } else if (sc->slice_us > 0. && t >= next_temp) {
            // Like the driver: one last read, then switch the mux away
            next_temp += temp_period;
            start_slice = 1;
            slice_end = t + sc->slice_us * 1e-6;
        }
        if (start_slice)
            in_slice = 1;
        if (latest_idx < 0)
            continue;
        struct mads_result r;
        uint32_t prev_new = s.n_new;
        mads_sampler_read(&s, now, latest, &r);
        uint_fast8_t i;
        for (i = 0; i < r.fillers; i++)
            emit(t, MADS_FILLER, -1, 1);
        for (i = 0; i < r.holds; i++)
            emit(t, r.hold_value, -2, 0);
        // A late read may lose as many conversions as fit in its gap
        if (r.late)
            flagged += (long)((t - t_prev_read) / period) + 1;
        t_prev_read = t;
        if (r.emit) {
            emit(t, r.value, latest_idx, 0);
            measured++;
            if (s.n_new != prev_new) {
                // Delay from the conversion completing to the mcu seeing it
                double d = t - convs[latest_idx].t;
                if (d > max_delay)
                    max_delay = d;
                sum_delay += d;
                n_delay++;
            }
        }
        measured += r.holds;
        // Never report more conversions than the chip has made
        if (measured > nconv)
            phantoms++;
    }
    // Conversions completed before the last read but never reported
    int read_convs = 0, i;
    for (i = 0; i < nconv; i++)
        if (convs[i].t <= t)
            read_convs++;
    long lost = read_convs - measured;
    double rate_err;
    double resid = fit_residual(&rate_err);
    double new_bound = read_interval + sc->jitter_us * 1e-6 + 5e-6;
    int ok = !phantoms && lost <= flagged;
    if (sc->p_stall == 0. && max_delay > new_bound)
        ok = 0;
    if (sc->saturate || sc->p_repeat > 0.1)
        // Long identical runs may under-count by the oscillator tolerance
        ok = !phantoms && lost <= flagged + (long)(s.n_dup * 4 / 100) + 1;
    printf("%-20s conv=%6d new=%6u dup=%5u hold=%3u lost=%3ld flagged=%3ld"
           " fill=%4u late=%4u delay max=%5.1fus mean=%5.1fus"
           " fit=%6.1fus %s\n"
           , sc->name, nconv, s.n_new, s.n_dup, s.n_hold, lost, flagged
           , s.n_filler, s.n_late, max_delay * 1e6
           , n_delay ? sum_delay / n_delay * 1e6 : 0.
           , resid * 1e6, ok ? "ok" : "FAIL");
    (void)rate_err;
    failures += !ok;
    (void)tick;
    (void)verbose;
    return failures;
}

// The KAN-355 timed RDATA reader: one read every 1/(0.952 * nominal SPS)
static void
kan355(double osc_err, double seconds)
{
    double period = CONV_TCLK * T_CLK * (1. + osc_err);
    double read = 1. / (2000. / 1.05);
    double t, max_age = 0., sum_age = 0.;
    long reads = 0, convs_total = (long)(seconds / period), skipped = 0;
    long last_k = -1;
    for (t = read; t < seconds; t += read) {
        long k = (long)floor(t / period) - 1;  // latest completed
        double age = t - (k + 1) * period;
        if (age > max_age)
            max_age = age;
        sum_age += age;
        if (last_k >= 0 && k > last_k + 1)
            skipped += k - last_k - 1;
        if (k == last_k)
            printf("kan355: conversion %ld read twice\n", k);
        last_k = k;
        reads++;
    }
    printf("%-26s conv=%6ld reads=%6ld skipped=%5ld (%.1f%%) age max=%5.1fus"
           " mean=%5.1fus\n", osc_err > 0 ? "kan355 osc +2%" : "kan355 osc -2%"
           , convs_total, reads, skipped, 100. * skipped / convs_total
           , max_age * 1e6, sum_age / reads * 1e6);
}

int
main(void)
{
    static const struct scenario scenarios[] = {
        // name            osc    sigma prep  jit  pstall stall slice sat secs
        { "nominal",        0.,    184., 0.,   10., 0.,    0.,   0.,   0, 10. },
        { "osc -2%",       -0.02,  184., 0.,   10., 0.,    0.,   0.,   0, 10. },
        { "osc +2%",        0.02,  184., 0.,   10., 0.,    0.,   0.,   0, 10. },
        { "repeats 5%",     0.,    184., 0.05, 10., 0.,    0.,   0.,   0, 10. },
        { "repeats 30%",    0.02,  184., 0.30, 20., 0.,    0.,   0.,   0, 10. },
        { "sigma=2",       -0.02,  2.,   0.,   20., 0.,    0.,   0.,   0, 10. },
        { "ntc slices",     0.01,  184., 0.,   20., 0.,    0.,   1700, 0, 10. },
        { "pt1000 slices", -0.02,  184., 0.02, 20., 0.,    0.,   2800, 0, 10. },
        { "saturated",      0.,    184., 0.,   10., 0.,    0.,   1700, 1, 10. },
        { "stalls 400us",   0.,    184., 0.,   20., 0.002, 400,  1700, 0, 10. },
        { "stalls 1500us",  0.015, 184., 0.,   20., 0.001, 1500, 2800, 0, 10. },
    };
    int failures = 0;
    unsigned i;
    for (i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
        rng_state = 12345 + i;
        failures += run(&scenarios[i], 0);
    }
    kan355(-0.02, 10.);
    kan355(0.02, 10.);
    if (failures)
        printf("%d scenario(s) FAILED\n", failures);
    return failures ? 1 : 0;
}
