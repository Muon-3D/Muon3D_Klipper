// Exercise the actual cross-MCU dispatcher. Transport/clock are test doubles.
#include <assert.h>
#include <stdio.h>
#include "../../klippy/chelper/trdispatch.c"
static int sent, reasons[8];
void report_errno(char *where, int rc) { assert(0); }
struct queue_message *message_alloc_and_encode(uint32_t *data, int len) {
    struct queue_message *q=calloc(1,sizeof(*q));
    q->len=len*sizeof(uint32_t); memcpy(q->msg,data,q->len); return q;
}
int msgblock_decode(uint32_t *data,int n,uint8_t *msg,int len) {
    if(len!=n*sizeof(uint32_t)) return -1;
    memcpy(data,msg,len); return 0;
}
void serialqueue_send_one(struct serialqueue *s,struct command_queue *c,
                          struct queue_message *q) {
    uint32_t fields[3]; memcpy(fields,q->msg,sizeof(fields));
    assert(sent<8); reasons[sent++]=fields[2]; free(q);
}
void serialqueue_get_clock_est(struct serialqueue *s,struct clock_estimate *c) {
    c->est_freq=12000000.; c->conv_clock=0; c->conv_time=0.;
}
void serialqueue_add_fastreader(struct serialqueue *s,struct fastreader *f) {}
void serialqueue_rm_fastreader(struct serialqueue *s,struct fastreader *f) {}
uint64_t clock_from_clock32(uint64_t c,uint32_t low) { return low; }
uint64_t clock_from_time(struct clock_estimate *c,double t) { return t*c->est_freq; }
double clock_to_time(struct clock_estimate *c,uint64_t t) { return t/c->est_freq; }
static void run(struct trdispatch *td,struct trdispatch_mcu *source,
                int incoming,int optin,int expected) {
    sent=0; trdispatch_start(td,2);
    uint32_t fields[]={12,source->trsync_oid,0,incoming,12000};
    handle_trsync_state(&source->fr,.001,(uint8_t*)fields,sizeof(fields));
    assert(sent==2 && reasons[0]==expected && reasons[1]==expected);
    handle_trsync_state(&source->fr,.001,(uint8_t*)fields,sizeof(fields));
    assert(sent==2); // A second message must not authorize a second lift.
    trdispatch_stop(td);
}
int main(void) {
    struct trdispatch *td=trdispatch_alloc();
    struct trdispatch_mcu *sensor=trdispatch_mcu_alloc(td,NULL,NULL,1,10,11,12);
    struct trdispatch_mcu *motor=trdispatch_mcu_alloc(td,NULL,NULL,2,10,11,12);
    for(int opt=0;opt<2;opt++) {
        trdispatch_set_retract(td,sensor,opt ? 255 : 0);
        for(int reason=0;reason<256;reason++) {
            run(td,sensor,reason,opt,opt && reason==1 ? 255 : 2);
            run(td,motor,reason,opt,2);
        }
    }
    // Reconnect can reorder the list; identity, not list order, authorizes lift.
    trdispatch_mcu_clear(sensor); free(sensor);
    sensor=trdispatch_mcu_alloc(td,NULL,NULL,1,10,11,12);
    trdispatch_set_retract(td,sensor,255);
    run(td,sensor,1,1,255); run(td,motor,1,1,2);
    trdispatch_mcu_clear(sensor); trdispatch_mcu_clear(motor);
    free(sensor); free(motor); pthread_mutex_destroy(&td->lock); free(td);
    puts("Actual trdispatch.c reason-routing tests passed"); return 0;
}
