"""Run real buttons.c against GPIO/scheduler stubs; no firmware build."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
STUB = r'''
#define __BASECMD_H
#define __COMMAND_H
#define __SCHED_H
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
struct timer { uint32_t waketime; uint_fast8_t (*func)(struct timer *); };
struct gpio_in { uint8_t pin; };
struct task_wake { int awake; };
#define SF_RESCHEDULE 1
#define container_of(p,t,m) ((t *)((char *)(p) - offsetof(t,m)))
#define DECL_COMMAND(f,s)
#define DECL_TASK(f)
#define foreach_oid(oid,b,fn) for (oid=0,b=obj; oid<1; oid++)
static void *obj;
static uint8_t pin_state;
static int reports, reported_state;
static void *oid_alloc(int oid, void *fn, size_t size) {
    return obj = calloc(1, size);
}
static void *oid_lookup(int oid, void *fn) { return obj; }
static void shutdown(const char *msg) { abort(); }
static struct gpio_in gpio_in_setup(int pin, int pull) {
    return (struct gpio_in){pin};
}
static int gpio_in_read(struct gpio_in pin) { return pin_state; }
static void irq_disable(void) {}
static void irq_enable(void) {}
static void sched_add_timer(struct timer *t) {}
static void sched_del_timer(struct timer *t) {}
static void sched_wake_task(struct task_wake *t) { t->awake = 1; }
static int sched_check_wake(struct task_wake *t) {
    int awake = t->awake; t->awake = 0; return awake;
}
static void sendf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    va_arg(ap, int); va_arg(ap, int);
    int count = va_arg(ap, int);
    uint8_t *states = va_arg(ap, uint8_t *);
    reports += count; reported_state = states[count-1];
    va_end(ap);
}
'''
MAIN = r'''
int main(int argc, char **argv) {
    pin_state = atoi(argv[1]);
    uint32_t config[] = {0, 1}, add[] = {0, 0, 3, 0};
    uint32_t query[] = {0, 100, 2, 50, pin_state};
    command_config_buttons(config);
    command_buttons_add(add);
    command_buttons_query(query);
    struct buttons *b = obj;
    buttons_event(&b->time);
    buttons_event(&b->time);
    buttons_task();
    if (reports != 1 || reported_state != pin_state) return 1;
    command_buttons_ack((uint32_t[]){0, 1});
    buttons_event(&b->time); buttons_task();
    if (reports != 1) return 2;
    /* A second setup must report an unchanged pin again. */
    command_buttons_query(query);
    buttons_event(&b->time); buttons_event(&b->time); buttons_task();
    return reports != 2;
}
'''


class FirstReportTest(unittest.TestCase):
    def test_unchanged_pin_reports_first_sample_after_each_setup(self):
        with tempfile.TemporaryDirectory(prefix='kan558-buttons-') as tmp:
            tmp = pathlib.Path(tmp)
            for name in ('basecmd.h', 'board/gpio.h', 'board/irq.h',
                         'command.h', 'sched.h'):
                path = tmp / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('')
            source = tmp / 'buttons_test.c'
            source.write_text(STUB + '\n#include "%s"\n' %
                              (ROOT / 'src/buttons.c') + MAIN)
            exe = tmp / 'buttons_test'
            subprocess.run(['gcc', '-I', str(tmp), str(source),
                            '-o', str(exe)], check=True)
            for state in ('0', '1'):
                with self.subTest(state=state):
                    result = subprocess.run([str(exe), state])
                    self.assertEqual(result.returncode, 0,
                                     'unchanged first sample was silent')


if __name__ == '__main__':
    unittest.main()
