// Execute the actual stepper.c with fake GPIO, clock, and scheduler.
// These are functional tests, not a substitute for MCU timing validation.
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/compiler.h"
#include "../../src/basecmd.h"
#define __COMMAND_H
#define __SCHED_H
#define DECL_COMMAND(f,s)
#define DECL_CONSTANT(s,v)
#define DECL_SHUTDOWN(f)
#define sendf(...) ((void)0)
static jmp_buf failure;
static int expecting_failure;
static void shutdown(const char *msg) {
    if (expecting_failure) longjmp(failure, 1);
    fprintf(stderr, "Unexpected shutdown: %s\n", msg);
    abort();
}
struct timer {
    struct timer *next;
    uint_fast8_t (*func)(struct timer *);
    uint32_t waketime;
};
enum { SF_DONE, SF_RESCHEDULE };
void sched_add_timer(struct timer *t);
void sched_del_timer(struct timer *t);
#include "../../src/stepper.c"

static struct timer *pending[4];
static uint32_t now;
static int pins[4], edge_count, pulse_count, both_edges, active_level;
static void *object;
static int object_size;
uint32_t timer_read_time(void) { return now++; }
struct gpio_out gpio_out_setup(unsigned int p, unsigned int value) {
    pins[p] = !!value;
    return (struct gpio_out){p};
}
void gpio_out_write(struct gpio_out p, unsigned int v) { pins[p.pin] = !!v; }
void gpio_out_toggle_noirq(struct gpio_out p) {
    pins[p.pin] ^= 1;
    if (p.pin == 1) {
        edge_count++;
        if (both_edges || pins[1] == active_level) pulse_count++;
    }
}
void *alloc_chunk(size_t n) { return calloc(1, n); }
void *oid_alloc(uint8_t oid, void *type, uint16_t n) {
    assert(!object); return object = calloc(1, n);
}
void *oid_lookup(uint8_t oid, void *type) { assert(oid == 0); return object; }
void *oid_next(uint8_t *i, void *type) {
    if (*i != 255) return NULL;
    *i = 0; return object;
}
void move_queue_setup(struct move_queue_head *q, int n) { object_size = n; }
void *move_alloc(void) { return calloc(1, object_size); }
void move_free(void *p) { free(p); }
int move_queue_empty(struct move_queue_head *q) { return !q->first; }
int move_queue_push(struct move_node *n, struct move_queue_head *q) {
    n->next = NULL;
    if (q->first) q->last->next = n; else q->first = n;
    q->last = n; return 0;
}
struct move_node *move_queue_pop(struct move_queue_head *q) {
    struct move_node *n = q->first; assert(n); q->first = n->next; return n;
}
void move_queue_clear(struct move_queue_head *q) {
    while (q->first) free(move_queue_pop(q));
}
struct trsync *trsync_oid_lookup(uint8_t oid) { return NULL; }
void trsync_add_signal(struct trsync *ts, struct trsync_signal *s,
                       trsync_callback_t cb) { s->func = cb; }
void sched_add_timer(struct timer *t) {
    for (int i=0; i<4; i++) if (!pending[i]) { pending[i]=t; return; }
    assert(0);
}
void sched_del_timer(struct timer *t) {
    for (int i=0; i<4; i++) if (pending[i]==t) pending[i]=NULL;
}
static int tick(void) {
    struct timer *t = NULL;
    for (int i=0; i<4; i++)
        if (pending[i] && (!t || timer_is_before(pending[i]->waketime,t->waketime)))
            t = pending[i];
    if (!t) return 0;
    sched_del_timer(t);
    now = t->waketime;
    int ret = t->func ? t->func(t) : stepper_event(t);
    if (ret == SF_RESCHEDULE) sched_add_timer(t);
    return 1;
}
static void drain(void) {
    int n=0; while (tick()) assert(++n < 20000);
}
static int32_t position(struct stepper *s) {
    return (int32_t)(stepper_get_position(s) - POSITION_BIAS);
}
#define CALL(fn,...) do { uint32_t a[]={__VA_ARGS__}; fn(a); } while(0)
#define REJECT(expr) do { \
    expecting_failure=1; \
    if (!setjmp(failure)) { expr; assert(!"expected shutdown"); } \
    expecting_failure=0; \
} while(0)
static struct stepper *fresh(int invert) {
    if (object) {
        struct stepper *s=object;
        move_queue_clear(&s->mq);
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
        if (s->retract) { free(s->retract->segments); free(s->retract); }
#endif
        free(object); object=NULL;
    }
    memset(pending,0,sizeof(pending));
    now=120000; edge_count=pulse_count=0;
    both_edges=invert<0; active_level=invert>0 ? 0 : 1;
    CALL(command_config_stepper,0,1,2,invert,2);
    CALL(command_reset_step_clock,0,now);
    return object;
}
static void ordinary(int invert) {
    struct stepper *s=fresh(invert);
    CALL(command_set_next_step_dir,0,1);
    CALL(command_queue_step,0,120,10,2);
    CALL(command_set_next_step_dir,0,0);
    CALL(command_queue_step,0,150,4,-2);
    drain(); assert(position(s)==6); assert(pulse_count==14);
}
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
static void arm(int dir, int reason) {
    CALL(command_stepper_stop_on_trigger,0,0);
    CALL(command_stepper_retract_arm,0,1,12000,dir,reason);
}
static void profile(void) {
    CALL(command_config_stepper_retract,0,512);
    CALL(command_stepper_retract_segment,0,0,1200,3,-100);
    CALL(command_stepper_retract_segment,0,1,1000,4,100);
}
static void retract_test(int invert,int dir,int reason,int interrupted_edges) {
    struct stepper *s=fresh(invert); profile();
    CALL(command_set_next_step_dir,0,!dir);
    CALL(command_queue_step,0,2400,100,0);
    arm(dir,reason);
    for (int i=0;i<interrupted_edges;i++) assert(tick());
    int32_t halt=position(s); int olddir=pins[2];
    stepper_stop(&s->stop_signal,reason);
    assert(pins[2]==olddir); // No reversal before hold delay.
    assert(s->retract->state==RS_ACTIVE);
    assert(position(s)==halt);
    REJECT(CALL(command_reset_step_clock,0,now));
    CALL(command_queue_step,0,100,99,0); // Late descent must be discarded.
    int initial_pulses=pulse_count;
    drain();
    assert(s->retract->state==RS_DONE);
    int32_t end=halt+(dir ? 7 : -7);
    assert(position(s)==end); assert(pulse_count-initial_pulses==7);
    assert(s->retract->end_clock-s->retract->start_clock>=7900);
    now+=100;
    CALL(command_reset_step_clock,0,now);
    assert(position(s)==end); assert(pins[2]==0);
    CALL(command_set_next_step_dir,0,1);
    CALL(command_queue_step,0,120,5,0);
    drain(); assert(position(s)==end+5);
    // A later ordinary reset must not corrupt the direction encoding.
    now+=100; CALL(command_reset_step_clock,0,now);
    CALL(command_queue_step,0,120,2,0);
    drain(); assert(position(s)==end+7);
    arm(dir,reason); stepper_stop(&s->stop_signal,reason); drain();
    assert(position(s)==end+7+(dir ? 7 : -7));
}
static void failures(void) {
    for (int reason=0;reason<256;reason++) {
        if (reason==1) continue;
        struct stepper *s=fresh(0); profile(); arm(1,1);
        stepper_stop(&s->stop_signal,reason); drain();
        assert(s->retract->state==RS_CANCELLED && !pulse_count);
    }
    for (int stage=0;stage<3;stage++) {
        struct stepper *s=fresh(0); profile(); arm(1,1);
        if (stage) stepper_stop(&s->stop_signal,1);
        if (stage==2) { tick(); tick(); }
        CALL(command_stepper_retract_cancel,0);
        int p=pulse_count; drain(); assert(pulse_count==p);
        assert(s->retract->state==RS_CANCELLED);
    }
    fresh(0); profile();
    REJECT(CALL(command_stepper_retract_segment,0,2,1200,0,0));
    REJECT(CALL(command_stepper_retract_segment,0,9,1200,3,0));
    REJECT(CALL(command_stepper_retract_segment,0,2,1,3,0));
    REJECT(CALL(command_stepper_retract_segment,0,2,1200,4096,0));
    REJECT(CALL(command_stepper_retract_segment,0,2,7000000,1,0));
    REJECT(CALL(command_stepper_retract_segment,0,2,50,3,-100));
    REJECT(CALL(command_stepper_retract_arm,0,0,12000,1,1));
    REJECT(CALL(command_stepper_retract_arm,0,1,1,1,1));
    REJECT(CALL(command_stepper_retract_arm,0,1,12000,2,1));
    REJECT(CALL(command_stepper_retract_arm,0,1,12000,1,5));
    arm(1,1);
    REJECT(CALL(command_stepper_retract_segment,0,0,1200,3,0));
    REJECT(CALL(command_stepper_retract_arm,0,1,12000,1,1));
    stepper_shutdown(); drain(); assert(!pulse_count);
    // Both the delayed start and the ascent may cross the 32-bit wrap.
    struct stepper *s=fresh(0); profile();
    now=0xfffff000; CALL(command_reset_step_clock,0,now);
    arm(1,1); stepper_stop(&s->stop_signal,1); drain();
    assert(position(s)==7 && s->retract->state==RS_DONE);
}
#endif
int main(void) {
    for(int invert=-1;invert<=1;invert++) {
        ordinary(invert);
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
        for(int dir=0;dir<2;dir++) for(int edges=0;edges<5;edges++) {
            retract_test(invert,dir,1,edges);
            retract_test(invert,dir,255,edges);
        }
#endif
    }
#if CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT
    failures();
#endif
    puts("Actual stepper.c motion/state tests passed"); return 0;
}
