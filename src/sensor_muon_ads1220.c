// Muon B1 toolhead ADS1220: load cell and hotend temperature on one chip
//
// Copyright (C) 2026  Muon3D
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The chip's DRDY pin is not connected, so conversions are found by reading
// RDATA at ~2x the conversion rate (see sensor_muon_ads1220.h). The chip
// streams the load cell, and every temperature period the mcu switches the
// input mux to the hotend sensor for one or two settled conversions and
// back. Mux changes are single register writes; the excitation (IDAC or the
// high-side switch) is fixed per hotend mode and never changes while
// running, so the sensor's input filter never has to re-settle.
//
// While a load cell probe trigger is armed the temperature reads pause, so
// the probe never goes blind; the host holds the heater off for the same
// window. The temperature is range checked here, so an open or shorted
// sensor shuts the mcu down without waiting for the host.
//
// All SPI traffic is issued from task context, like every other device on
// the bus, so it can never interleave with another device's transfer.

#include <stdint.h>
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "sensor_muon_ads1220.h" // mads_sampler_read
#include "spicmds.h" // spidev_transfer
#include "trigger_analog.h" // trigger_analog_update

// ADS1220 commands
enum {
    CMD_POWERDOWN = 0x02, CMD_RESET = 0x06, CMD_START = 0x08,
    CMD_RDATA = 0x10, CMD_RREG = 0x20, CMD_WREG = 0x40,
};

// Hotend sensor excitation
enum { MODE_OFF, MODE_PT1000, MODE_NTC, MODE_COUNT };

// What the chip is converting
enum {
    ST_IDLE, ST_LC_SETTLE, ST_LC, ST_TEMP_SENSE, ST_TEMP_REF,
};

// Flags
enum {
    MF_LC = 1<<0, MF_RUNNING = 1<<1, MF_PAUSED = 1<<2, MF_BLIND = 1<<3,
    MF_TOLD = 1<<4,
};

// Sensor specific errors reported through trigger_analog
enum { ME_STALL };

#define BYTES_PER_SAMPLE 4
#define ADS1220_SAT_MAX 0x7fffff
#define ADS1220_SAT_MIN (-0x800000)

struct muon_ads1220 {
    struct timer timer;
    struct spidev_s *spi;
    struct trigger_analog *ta;
    struct gpio_out high_side;
    uint8_t high_side_off;
    uint8_t pending, flags, mode, state;
    // Register images
    uint8_t lc_reg0, reg1, cur_reg0;
    uint8_t reg2[MODE_COUNT], reg3[MODE_COUNT];
    uint8_t sense_reg0[MODE_COUNT], ref_reg0[MODE_COUNT];
    // Timing, in mcu ticks
    uint32_t read_ticks, settle_ticks, lc_first_ticks, lc_settle_ticks;
    uint32_t temp_ticks, max_pause_ticks;
    uint16_t stall_count;
    uint32_t state_time, next_temp_time, pause_start;
    // Temperature reading in progress
    int32_t sense;
    uint32_t sense_time;
    // Range check: NTC checks sense; PT1000 checks ref and sense/ref (Q12)
    int32_t range_min, range_max, ref_min, ref_max;
    uint8_t fault_max, fault_count;
    // Load cell stream
    struct sensor_bulk sb;
    struct mads_sampler ms;
    // Statistics
    uint32_t n_temp, n_blind, n_overrun, n_stall;
};

static struct task_wake wake_muon_ads1220;

void command_config_muon_ads1220(uint32_t *args);


/****************************************************************
 * Chip access
 ****************************************************************/

static void
ads_command(struct muon_ads1220 *a, uint8_t cmd)
{
    spidev_transfer(a->spi, 0, 1, &cmd);
}

static void
ads_write_regs(struct muon_ads1220 *a, const uint8_t *regs)
{
    uint8_t msg[5] = { CMD_WREG | 0x03, regs[0], regs[1], regs[2], regs[3] };
    spidev_transfer(a->spi, 0, sizeof(msg), msg);
    a->cur_reg0 = regs[0];
}

// Change the input mux/gain register. This restarts the conversion.
static uint32_t
ads_write_reg0(struct muon_ads1220 *a, uint8_t reg0)
{
    uint8_t msg[2] = { CMD_WREG | 0x00, reg0 };
    spidev_transfer(a->spi, 0, sizeof(msg), msg);
    a->cur_reg0 = reg0;
    return timer_read_time();
}

// Compare all four configuration registers against what was written
static int
ads_check_regs(struct muon_ads1220 *a)
{
    uint8_t msg[5] = { CMD_RREG | 0x03, 0, 0, 0, 0 };
    spidev_transfer(a->spi, 1, sizeof(msg), msg);
    return (msg[1] == a->cur_reg0 && msg[2] == a->reg1
            && msg[3] == a->reg2[a->mode] && msg[4] == a->reg3[a->mode]);
}

// Most recent conversion result
static int32_t
ads_read_data(struct muon_ads1220 *a)
{
    uint8_t msg[4] = { CMD_RDATA, 0, 0, 0 };
    spidev_transfer(a->spi, 1, sizeof(msg), msg);
    uint32_t counts = ((uint32_t)msg[1] << 16) | ((uint32_t)msg[2] << 8)
                      | msg[3];
    if (counts & 0x800000)
        counts |= 0xff000000;
    return counts;
}

static void
high_side_set(struct muon_ads1220 *a, uint8_t on)
{
    gpio_out_write(a->high_side, on ? !a->high_side_off : a->high_side_off);
}


/****************************************************************
 * Load cell stream
 ****************************************************************/

static void
add_sample(struct muon_ads1220 *a, uint8_t oid, int32_t counts)
{
    uint8_t *d = &a->sb.data[a->sb.data_count];
    d[0] = counts;
    d[1] = counts >> 8;
    d[2] = counts >> 16;
    d[3] = counts >> 24;
    a->sb.data_count += BYTES_PER_SAMPLE;
    if (a->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(a->sb.data))
        sensor_bulk_report(&a->sb, oid);
}

static void
emit_fillers(struct muon_ads1220 *a, uint8_t oid, uint32_t count)
{
    while (count--)
        add_sample(a, oid, MADS_FILLER);
}

static void
emit_result(struct muon_ads1220 *a, uint8_t oid, struct mads_result *r)
{
    emit_fillers(a, oid, r->fillers);
    if (r->late)
        // Reads fell further apart than a conversion: one may be lost
        a->sb.possible_overflows++;
    uint_fast8_t i;
    for (i = 0; i < r->holds; i++) {
        trigger_analog_update(a->ta, r->hold_value);
        add_sample(a, oid, r->hold_value);
    }
    if (r->emit) {
        trigger_analog_update(a->ta, r->value);
        add_sample(a, oid, r->value);
    }
}

// The chip is measuring temperature: keep the stream's slots ticking
static void
fill_gap(struct muon_ads1220 *a, uint8_t oid, uint32_t now)
{
    if (!(a->flags & MF_LC))
        return;
    emit_fillers(a, oid, mads_sampler_gap_fill(&a->ms, now));
    if (a->flags & MF_BLIND)
        // Paused past max_pause while armed: the sensor is alive, and the
        // host has been told this slice happens (see n_blind)
        trigger_analog_note_alive(a->ta);
}

static void
resume_load_cell(struct muon_ads1220 *a)
{
    uint32_t w = ads_write_reg0(a, a->lc_reg0);
    mads_sampler_restart(&a->ms, w + a->lc_first_ticks);
    a->state = ST_LC_SETTLE;
    a->state_time = w + a->lc_settle_ticks;
    a->flags &= ~MF_BLIND;
}


/****************************************************************
 * Temperature
 ****************************************************************/

static int
is_armed(struct muon_ads1220 *a)
{
    return (a->flags & MF_LC) && trigger_analog_is_armed(a->ta);
}

// Check if a temperature slice should start now
static int
temp_is_due(struct muon_ads1220 *a, uint8_t oid, uint32_t now)
{
    if (mads_before(now, a->next_temp_time))
        return 0;
    if (is_armed(a)) {
        if (!(a->flags & MF_PAUSED)) {
            a->flags |= MF_PAUSED;
            a->pause_start = now;
        }
        if (mads_before(now, a->pause_start + a->max_pause_ticks))
            return 0;
        if (!(a->flags & MF_TOLD))
            // Once per armed window
            sendf("muon_ads1220_blind oid=%c", oid);
        a->flags |= MF_BLIND | MF_TOLD;
        a->n_blind++;
    } else {
        a->flags &= ~(MF_PAUSED | MF_BLIND | MF_TOLD);
    }
    return 1;
}

static void
start_temp(struct muon_ads1220 *a, uint32_t now)
{
    a->next_temp_time += a->temp_ticks;
    if (mads_before(a->next_temp_time, now))
        a->next_temp_time = now + a->temp_ticks;
    uint32_t w = ads_write_reg0(a, a->sense_reg0[a->mode]);
    a->state = ST_TEMP_SENSE;
    a->state_time = w + a->settle_ticks;
}

static void
check_range(struct muon_ads1220 *a, int32_t ref)
{
    if (!a->fault_max)
        return;
    int32_t sense = a->sense;
    int ok;
    if (a->mode == MODE_PT1000) {
        // sense/ref is the sensor resistance over the reference resistor
        if (ref < a->ref_min || ref > a->ref_max || sense < 0) {
            ok = 0;
        } else {
            uint32_t s = (uint32_t)sense << 4, r = (uint32_t)ref >> 8;
            ok = (s >= (uint32_t)a->range_min * r
                  && s <= (uint32_t)a->range_max * r);
        }
    } else {
        ok = sense >= a->range_min && sense <= a->range_max;
    }
    if (ok) {
        a->fault_count = 0;
        return;
    }
    a->fault_count++;
    if (a->fault_count >= a->fault_max)
        shutdown("ADS1220 hotend temperature out of range");
}

static void
finish_temp(struct muon_ads1220 *a, uint8_t oid, int32_t ref)
{
    // A chip that reset or lost its configuration would go on returning
    // plausible-looking data; check it on every temperature read.
    if (!ads_check_regs(a))
        shutdown("ADS1220 configuration lost");
    a->n_temp++;
    sendf("muon_ads1220_temp oid=%c clock=%u sense=%i ref=%i mode=%c"
          , oid, a->sense_time, a->sense, ref, a->mode);
    check_range(a, ref);
}


/****************************************************************
 * Scheduling
 ****************************************************************/

static uint_fast8_t
muon_ads1220_event(struct timer *timer)
{
    struct muon_ads1220 *a = container_of(timer, struct muon_ads1220, timer);
    if (a->pending) {
        a->n_overrun++;
    } else {
        a->pending = 1;
        sched_wake_task(&wake_muon_ads1220);
    }
    a->timer.waketime += a->read_ticks;
    return SF_RESCHEDULE;
}

static void
step(struct muon_ads1220 *a, uint8_t oid)
{
    uint32_t now = timer_read_time();
    switch (a->state) {
    case ST_IDLE:
        if (a->flags & MF_LC)
            resume_load_cell(a);
        else if (temp_is_due(a, oid, now))
            start_temp(a, now);
        break;
    case ST_LC_SETTLE:
    case ST_LC: {
        if (!(a->flags & MF_LC)) {
            a->state = ST_IDLE;
            break;
        }
        int temp_due = 0;
        if (a->state == ST_LC_SETTLE) {
            fill_gap(a, oid, now);
            if (mads_before(now, a->state_time))
                break;
        } else {
            temp_due = temp_is_due(a, oid, now);
        }
        // Always read before switching away, so a conversion that just
        // completed is not lost to the slice
        int32_t raw = ads_read_data(a);
        struct mads_result r;
        mads_sampler_read(&a->ms, now, raw, &r);
        emit_result(a, oid, &r);
        a->state = ST_LC;
        if (a->ms.dup_run >= a->stall_count && raw < ADS1220_SAT_MAX
            && raw > ADS1220_SAT_MIN) {
            // An unsaturated reading frozen for this long means the chip
            // stopped converting; do not let a probe rely on it
            a->n_stall++;
            a->ms.dup_run = 0;
            trigger_analog_note_error(a->ta, ME_STALL);
        }
        if (temp_due)
            start_temp(a, now);
        break;
    }
    case ST_TEMP_SENSE:
    case ST_TEMP_REF: {
        fill_gap(a, oid, now);
        if (is_armed(a) && !(a->flags & MF_BLIND)) {
            // A probe was armed during the slice: give it the load cell
            resume_load_cell(a);
            break;
        }
        if (mads_before(now, a->state_time))
            break;
        int32_t raw = ads_read_data(a);
        if (a->state == ST_TEMP_SENSE) {
            a->sense = raw;
            a->sense_time = now;
            if (a->mode == MODE_PT1000) {
                uint32_t w = ads_write_reg0(a, a->ref_reg0[a->mode]);
                a->state = ST_TEMP_REF;
                a->state_time = w + a->settle_ticks;
                break;
            }
            raw = 0;
        }
        finish_temp(a, oid, raw);
        if (a->flags & MF_LC)
            resume_load_cell(a);
        else
            a->state = ST_IDLE;
        break;
    }
    }
}

void
muon_ads1220_task(void)
{
    if (!sched_check_wake(&wake_muon_ads1220))
        return;
    uint8_t oid;
    struct muon_ads1220 *a;
    foreach_oid(oid, a, command_config_muon_ads1220) {
        if (!a->pending)
            continue;
        a->pending = 0;
        barrier();
        if (a->flags & MF_RUNNING)
            step(a, oid);
    }
}
DECL_TASK(muon_ads1220_task);


/****************************************************************
 * Commands
 ****************************************************************/

void
command_config_muon_ads1220(uint32_t *args)
{
    struct muon_ads1220 *a = oid_alloc(args[0], command_config_muon_ads1220
                                       , sizeof(*a));
    a->timer.func = muon_ads1220_event;
    a->spi = spidev_oid_lookup(args[1]);
    a->high_side_off = !!args[3];
    a->high_side = gpio_out_setup(args[2], a->high_side_off);
}
DECL_COMMAND(command_config_muon_ads1220, "config_muon_ads1220 oid=%c"
             " spi_oid=%c high_side_pin=%u high_side_off_value=%c");

static struct muon_ads1220 *
lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_muon_ads1220);
}

void
command_muon_ads1220_set_regs(uint32_t *args)
{
    struct muon_ads1220 *a = lookup(args[0]);
    a->lc_reg0 = args[1];
    a->reg1 = args[2];
    a->reg2[MODE_PT1000] = args[3];
    a->reg3[MODE_PT1000] = args[4];
    a->sense_reg0[MODE_PT1000] = args[5];
    a->ref_reg0[MODE_PT1000] = args[6];
    a->reg2[MODE_NTC] = args[7];
    a->reg3[MODE_NTC] = args[8];
    a->sense_reg0[MODE_NTC] = args[9];
}
DECL_COMMAND(command_muon_ads1220_set_regs, "muon_ads1220_set_regs oid=%c"
             " lc_reg0=%c reg1=%c pt_reg2=%c pt_reg3=%c pt_sense_reg0=%c"
             " pt_ref_reg0=%c ntc_reg2=%c ntc_reg3=%c ntc_sense_reg0=%c");

void
command_muon_ads1220_set_timing(uint32_t *args)
{
    struct muon_ads1220 *a = lookup(args[0]);
    a->read_ticks = args[1];
    mads_sampler_init(&a->ms, args[2], args[3], args[4]);
}
DECL_COMMAND(command_muon_ads1220_set_timing, "muon_ads1220_set_timing oid=%c"
             " read_ticks=%u conv_ticks=%u conv_min_ticks=%u"
             " conv_max_ticks=%u");

void
command_muon_ads1220_set_schedule(uint32_t *args)
{
    struct muon_ads1220 *a = lookup(args[0]);
    a->settle_ticks = args[1];
    a->lc_first_ticks = args[2];
    a->lc_settle_ticks = args[3];
    a->temp_ticks = args[4];
    a->max_pause_ticks = args[5];
    a->stall_count = args[6];
}
DECL_COMMAND(command_muon_ads1220_set_schedule
             , "muon_ads1220_set_schedule oid=%c settle_ticks=%u"
             " lc_first_ticks=%u lc_settle_ticks=%u temp_ticks=%u"
             " max_pause_ticks=%u stall_count=%hu");

void
command_muon_ads1220_attach_trigger_analog(uint32_t *args)
{
    struct muon_ads1220 *a = lookup(args[0]);
    a->ta = trigger_analog_oid_lookup(args[1]);
}
DECL_COMMAND(command_muon_ads1220_attach_trigger_analog,
    "muon_ads1220_attach_trigger_analog oid=%c trigger_analog_oid=%c");

// Configure the hotend excitation and start converting (mode 0 stops)
void
command_muon_ads1220_start(uint32_t *args)
{
    uint8_t oid = args[0];
    struct muon_ads1220 *a = lookup(oid);
    uint8_t mode = args[1];
    sched_del_timer(&a->timer);
    a->pending = 0;
    a->flags &= MF_LC;
    a->fault_max = a->fault_count = 0;
    a->state = ST_IDLE;
    mads_sampler_reset(&a->ms);
    high_side_set(a, 0);
    if (mode != MODE_PT1000 && mode != MODE_NTC) {
        // Also turns off the IDAC
        a->mode = MODE_OFF;
        ads_command(a, CMD_POWERDOWN);
        return;
    }
    ads_command(a, CMD_RESET);
    // At least 50us + 32 * tCLK after RESET before the next command
    uint32_t end = timer_read_time() + timer_from_us(70);
    while (mads_before(timer_read_time(), end))
        ;
    a->mode = mode;
    uint8_t regs[4] = { a->sense_reg0[mode], a->reg1, a->reg2[mode]
                        , a->reg3[mode] };
    ads_write_regs(a, regs);
    if (!ads_check_regs(a)) {
        // Not responding: stay stopped and let the host refuse to heat
        a->mode = MODE_OFF;
        ads_command(a, CMD_POWERDOWN);
        sendf("muon_ads1220_error oid=%c code=%c", oid, 0);
        return;
    }
    high_side_set(a, mode == MODE_NTC);
    ads_command(a, CMD_START);
    uint32_t now = timer_read_time();
    a->state = ST_TEMP_SENSE;
    a->state_time = now + a->settle_ticks;
    a->next_temp_time = now + a->temp_ticks;
    a->flags |= MF_RUNNING;
    irq_disable();
    a->timer.waketime = timer_read_time() + a->read_ticks;
    sched_add_timer(&a->timer);
    irq_enable();
}
DECL_COMMAND(command_muon_ads1220_start, "muon_ads1220_start oid=%c mode=%c");

// Arm the mcu-side hotend range check (fault_count=0 disarms it)
void
command_muon_ads1220_set_range(uint32_t *args)
{
    struct muon_ads1220 *a = lookup(args[0]);
    a->range_min = args[1];
    a->range_max = args[2];
    a->ref_min = args[3];
    a->ref_max = args[4];
    a->fault_count = 0;
    a->fault_max = args[5];
}
DECL_COMMAND(command_muon_ads1220_set_range, "muon_ads1220_set_range oid=%c"
             " min_value=%i max_value=%i ref_min=%i ref_max=%i"
             " fault_count=%c");

// Start or stop streaming load cell samples
void
command_muon_ads1220_lc_enable(uint32_t *args)
{
    struct muon_ads1220 *a = lookup(args[0]);
    if (args[1]) {
        sensor_bulk_reset(&a->sb);
        mads_sampler_reset(&a->ms);
        a->flags |= MF_LC;
        if (a->state == ST_LC || a->state == ST_LC_SETTLE)
            // Start a fresh segment rather than continue a stale one
            a->state = ST_IDLE;
    } else {
        a->flags &= ~(MF_LC | MF_PAUSED | MF_BLIND | MF_TOLD);
    }
}
DECL_COMMAND(command_muon_ads1220_lc_enable
             , "muon_ads1220_lc_enable oid=%c enable=%c");

void
command_muon_ads1220_query_status(uint32_t *args)
{
    uint8_t oid = args[0];
    struct muon_ads1220 *a = lookup(oid);
    irq_disable();
    uint32_t start_t = timer_read_time();
    irq_enable();
    sensor_bulk_status(&a->sb, oid, start_t, 0, 0);
}
DECL_COMMAND(command_muon_ads1220_query_status
             , "muon_ads1220_query_status oid=%c");

void
command_muon_ads1220_query_stats(uint32_t *args)
{
    uint8_t oid = args[0];
    struct muon_ads1220 *a = lookup(oid);
    sendf("muon_ads1220_stats oid=%c new=%u dup=%u hold=%u filler=%u"
          " late=%u", oid, a->ms.n_new, a->ms.n_dup, a->ms.n_hold
          , a->ms.n_filler, a->ms.n_late);
    sendf("muon_ads1220_stats2 oid=%c temp=%u blind=%u overrun=%u stall=%u"
          " mode=%c", oid, a->n_temp, a->n_blind, a->n_overrun, a->n_stall
          , a->mode);
}
DECL_COMMAND(command_muon_ads1220_query_stats
             , "muon_ads1220_query_stats oid=%c");

void
muon_ads1220_shutdown(void)
{
    uint8_t oid;
    struct muon_ads1220 *a;
    foreach_oid(oid, a, command_config_muon_ads1220) {
        // The heater is off; drop the NTC excitation too
        high_side_set(a, 0);
        a->flags = 0;
        a->pending = 0;
    }
}
DECL_SHUTDOWN(muon_ads1220_shutdown);
