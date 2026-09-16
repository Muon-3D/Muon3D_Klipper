// Handling of stepper drivers.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // stepper_event
#include "trsync.h" // trsync_add_signal

DECL_CONSTANT("STEPPER_STEP_BOTH_EDGE", 1);

#if CONFIG_INLINE_STEPPER_HACK && CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE
 #define HAVE_EDGE_OPTIMIZATION 1
 #define HAVE_AVR_OPTIMIZATION 0
#elif CONFIG_INLINE_STEPPER_HACK && CONFIG_MACH_AVR
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 1
#else
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 0
#endif

struct stepper_move {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0 };

// Experimental bounded, preloaded retract.  No profile allocation is done
// in the trigger handler and ordinary motion cannot append to this profile.
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
struct retract_segment {
    uint32_t interval;
    int16_t add;
    uint16_t count;
};
struct stepper_retract {
    struct timer start_timer;
    struct stepper *stepper;
    struct retract_segment *segments;
    uint16_t capacity, used, index;
    uint32_t ticks, steps, start_clock, end_clock, delay_ticks;
    uint32_t halt_position, cycle;
    uint8_t state, direction, reason;
};
enum { RS_IDLE, RS_ARMED, RS_ACTIVE, RS_DONE, RS_CANCELLED };
#endif

struct stepper {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time, step_pulse_ticks;
    struct gpio_out step_pin, dir_pin;
    uint32_t position;
    struct move_queue_head mq;
    struct trsync_signal stop_signal;
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    struct stepper_retract *retract;
#endif
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_OPTIMIZED_PATH=1<<5, SF_HAVE_ADD=1<<6
};

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s)
{
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    struct stepper_retract *r = s->retract;
    uint_fast8_t retracting = r && r->state == RS_ACTIVE;
    if (retracting ? r->index == r->used : move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
        s->count = 0;
        if (retracting) {
            r->end_clock = s->time.waketime;
            r->state = RS_DONE;
        }
        return SF_DONE;
    }
#else
    if (move_queue_empty(&s->mq)) {
        s->count = 0;
        return SF_DONE;
    }
#endif

    // Read next 'struct stepper_move'
    uint32_t move_interval;
    uint_fast16_t move_count;
    int_fast16_t move_add;
    uint_fast8_t need_dir_change;
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    if (retracting) {
        struct retract_segment *m = &r->segments[r->index];
        move_interval = m->interval;
        move_count = m->count;
        move_add = m->add;
        need_dir_change = !r->index && r->direction;
        r->index++;
    } else
#endif
    {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        move_interval = m->interval;
        move_count = m->count;
        move_add = m->add;
        need_dir_change = m->flags & MF_DIR;
        move_free(m);
    }

    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    s->position = (need_dir_change ? -s->position : s->position) + move_count;

    // Load next move into 'struct stepper'
    s->add = move_add;
    s->interval = move_interval + move_add;
    if (HAVE_EDGE_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH) {
        // Using optimized stepper_event_edge()
        s->time.waketime += move_interval;
        s->count = move_count;
    } else if (HAVE_AVR_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH) {
        // Using optimized stepper_event_avr()
        s->time.waketime += move_interval;
        s->count = move_count;
        s->flags = move_add ? s->flags | SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
    } else {
        // Using fully scheduled stepper_event_full() code (the scheduler
        // may be called twice for each step)
        uint_fast8_t was_active = !!s->count;
        uint32_t min_next_time = s->time.waketime;
        s->next_step_time += move_interval;
        s->time.waketime = s->next_step_time;
        s->count = (s->flags & SF_SINGLE_SCHED ? move_count
                    : (uint32_t)move_count * 2);
        if (was_active && timer_is_before(s->next_step_time, min_next_time)) {
            // Actively stepping and next step event close to the last unstep
            int32_t diff = s->next_step_time - min_next_time;
            if (diff < (int32_t)-timer_from_us(1000))
                shutdown("Stepper too far in past");
            s->time.waketime = min_next_time;
        }
        if (was_active && need_dir_change) {
            // Must ensure minimum time between step change and dir change
            if (s->flags & SF_SINGLE_SCHED)
                while (timer_is_before(timer_read_time(), min_next_time))
                    ;
            gpio_out_toggle_noirq(s->dir_pin);
            uint32_t curtime = timer_read_time();
            min_next_time = curtime + s->step_pulse_ticks;
            if (timer_is_before(s->time.waketime, min_next_time))
                s->time.waketime = min_next_time;
            return SF_RESCHEDULE;
        }
    }

    // Set new direction (if needed)
    if (need_dir_change)
        gpio_out_toggle_noirq(s->dir_pin);
    return SF_RESCHEDULE;
}

// Edge optimization only enabled when fastest rate notably slower than 100ns
#define EDGE_STEP_TICKS DIV_ROUND_UP(CONFIG_CLOCK_FREQ, 8000000)
#if HAVE_EDGE_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_EDGE", EDGE_STEP_TICKS);
#endif

// Optimized step function to step on each step pin edge
static uint_fast8_t
stepper_event_edge(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
        return SF_RESCHEDULE;
    }
    return stepper_load_next(s);
}

#define AVR_STEP_TICKS 40 // minimum instructions between step gpio pulses
#if HAVE_AVR_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_UNSTEP", AVR_STEP_TICKS);
#endif

// AVR optimized step function
static uint_fast8_t
stepper_event_avr(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint16_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->time.waketime += s->interval;
        gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    gpio_out_toggle_noirq(s->step_pin);
    return ret;
}

// Regular "fully scheduled" step function
static uint_fast8_t
stepper_event_full(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t curtime = timer_read_time();
    uint32_t min_next_time = curtime + s->step_pulse_ticks;
    uint32_t count = s->count - 1;
    if (likely(count & 1 && !(s->flags & SF_SINGLE_SCHED)))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        s->count = count;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    s->time.waketime = min_next_time;
    return stepper_load_next(s);
reschedule_min:
    s->count = count;
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

// Optimized entry point for step function (may be inlined into sched.c code)
uint_fast8_t
stepper_event(struct timer *t)
{
    if (HAVE_EDGE_OPTIMIZATION)
        return stepper_event_edge(t);
    if (HAVE_AVR_OPTIMIZATION)
        return stepper_event_avr(t);
    return stepper_event_full(t);
}

void
command_config_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    int_fast8_t invert_step = args[3];
    if (invert_step > 0)
        s->flags = SF_INVERT_STEP;
    else if (invert_step < 0)
        s->flags = SF_SINGLE_SCHED;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = -POSITION_BIAS;
    s->step_pulse_ticks = args[4];
    move_queue_setup(&s->mq, sizeof(struct stepper_move));
    if (HAVE_EDGE_OPTIMIZATION) {
        if (invert_step < 0 && s->step_pulse_ticks <= EDGE_STEP_TICKS)
            s->flags |= SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (HAVE_AVR_OPTIMIZATION) {
        if (invert_step >= 0 && s->step_pulse_ticks <= AVR_STEP_TICKS)
            s->flags |= SF_SINGLE_SCHED | SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (!CONFIG_INLINE_STEPPER_HACK) {
        s->time.func = stepper_event_full;
    }
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid
static struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;

    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    if (flags & SF_NEED_RESET) {
        // Drop late descent packets even while the autonomous lift is active.
        move_free(m);
    } else if (s->count) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        stepper_load_next(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
static uint32_t stepper_get_position(struct stepper *s);

void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    if (s->retract && s->retract->state == RS_ACTIVE)
        shutdown("Can't reset time during retract");
    if (s->retract && s->retract->state == RS_ARMED)
        s->retract->state = RS_CANCELLED;
    if (s->retract && s->retract->state == RS_DONE) {
        // Restore the direction convention expected by stepcompress_reset().
        if (timer_is_before(timer_read_time(),
                            s->retract->end_clock + s->step_pulse_ticks))
            shutdown("Retract direction hold violation");
        gpio_out_write(s->dir_pin, 0);
        s->position = -stepper_get_position(s);
        s->flags &= ~(SF_LAST_DIR | SF_NEXT_DIR);
        s->retract->state = RS_IDLE;
    }
#endif
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (s->flags & SF_SINGLE_SCHED)
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Reverse only after the configured direction-hold delay has elapsed.
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
static uint_fast8_t
stepper_retract_start(struct timer *t)
{
    struct stepper_retract *r = container_of(t, struct stepper_retract,
                                            start_timer);
    struct stepper *s = r->stepper;
    r->start_clock = timer_read_time();
    s->next_step_time = s->time.waketime = r->start_clock;
    gpio_out_write(s->dir_pin, 0);
    stepper_load_next(s);
    sched_add_timer(&s->time);
    return SF_DONE;
}
#endif

// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    struct stepper_retract *r = s->retract;
    uint_fast8_t start_retract = (r && r->state == RS_ARMED
                                  && reason == r->reason);
    if (r && (r->state == RS_ARMED || r->state == RS_ACTIVE))
        r->state = RS_CANCELLED;
    if (r)
        sched_del_timer(&r->start_timer);
#endif
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = ((s->flags & (SF_INVERT_STEP|SF_SINGLE_SCHED|SF_OPTIMIZED_PATH))
                | SF_NEED_RESET);
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    if (!start_retract)
#endif
        gpio_out_write(s->dir_pin, 0);
    if (!(s->flags & SF_SINGLE_SCHED)
        || (HAVE_AVR_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH))
        // Must return step pin to "unstep" state
        gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        move_free(m);
    }
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    if (start_retract) {
        r->halt_position = stepper_get_position(s);
        r->start_timer.waketime = timer_read_time() + r->delay_ticks;
        r->index = 0;
        r->state = RS_ACTIVE;
        sched_add_timer(&r->start_timer);
    }
#endif
}

#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
void
command_config_stepper_retract(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t capacity = args[1];
    if (s->retract || !capacity || capacity > 512)
        shutdown("Invalid retract capacity");
    s->retract = alloc_chunk(sizeof(*s->retract));
    s->retract->stepper = s;
    s->retract->start_timer.func = stepper_retract_start;
    s->retract->capacity = capacity;
    s->retract->segments = alloc_chunk(capacity * sizeof(struct retract_segment));
}
DECL_COMMAND(command_config_stepper_retract,
             "config_stepper_retract oid=%c capacity=%hu");

void
command_stepper_retract_segment(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_retract *r = s->retract;
    uint32_t index = args[1], interval = args[2], count = args[3];
    int16_t add = args[4];
    if (!r || r->state == RS_ARMED || r->state == RS_ACTIVE
        || index >= r->capacity || !count || count > 4096)
        shutdown("Invalid retract segment");
    if (!index) {
        r->used = r->ticks = r->steps = 0;
        r->state = RS_IDLE;
    }
    int64_t last = (int64_t)interval + (int64_t)add * (count - 1);
    uint32_t min_ticks = 2 * s->step_pulse_ticks + timer_from_us(2);
    uint64_t ticks = ((int64_t)interval + last) * count / 2;
    if (index != r->used || interval < min_ticks || last < min_ticks
        || r->steps + count > 4096
        || ticks + r->ticks > timer_from_us(500000))
        shutdown("Retract profile exceeds limits");
    r->segments[index] = (struct retract_segment){ interval, add, count };
    r->used++;
    r->ticks += ticks;
    r->steps += count;
}
DECL_COMMAND(command_stepper_retract_segment, "stepper_retract_segment oid=%c"
             " index=%hu interval=%u count=%hu add=%hi");

void
command_stepper_retract_arm(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_retract *r = s->retract;
    uint32_t cycle = args[1], delay = args[2], reason = args[4];
    irq_disable();
    if (!r || !r->used || r->state == RS_ARMED || r->state == RS_ACTIVE
        || !cycle || delay < timer_from_us(1000)
        || delay > timer_from_us(10000) || args[3] > 1
        || (reason != 1 && reason != 255) || !s->stop_signal.func)
        shutdown("Invalid retract arm");
    r->cycle = cycle;
    r->delay_ticks = delay;
    r->direction = args[3];
    r->reason = reason;
    r->start_clock = r->end_clock = 0;
    r->state = RS_ARMED;
    irq_enable();
}
DECL_COMMAND(command_stepper_retract_arm, "stepper_retract_arm oid=%c cycle=%u"
             " delay=%u dir=%c reason=%c");

void
command_stepper_retract_query(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_retract *r = s->retract;
    if (!r)
        shutdown("Retract not configured");
    irq_disable();
    uint8_t state = r->state;
    uint32_t cycle = r->cycle, start = r->start_clock, end = r->end_clock;
    uint32_t halt = r->halt_position, pos = stepper_get_position(s);
    irq_enable();
    sendf("stepper_retract_state oid=%c cycle=%u state=%c"
          " start=%u end=%u halt=%i pos=%i", args[0], cycle, state,
          start, end, halt - POSITION_BIAS, pos - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_retract_query, "stepper_retract_query oid=%c");

void
command_stepper_retract_cancel(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    irq_disable();
    if (s->retract && (s->retract->state == RS_ARMED
                       || s->retract->state == RS_ACTIVE))
        stepper_stop(&s->stop_signal, 0);
    irq_enable();
}
DECL_COMMAND(command_stepper_retract_cancel, "stepper_retract_cancel oid=%c");
#endif

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_stepper_stop_on_trigger(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop);
}
DECL_COMMAND(command_stepper_stop_on_trigger,
             "stepper_stop_on_trigger oid=%c trsync_oid=%c");

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        move_queue_clear(&s->mq);
        stepper_stop(&s->stop_signal, 0);
    }
}
DECL_SHUTDOWN(stepper_shutdown);
