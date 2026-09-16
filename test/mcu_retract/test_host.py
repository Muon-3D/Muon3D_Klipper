import ast
import importlib.util
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'load_cell_retract', ROOT/'klippy/extras/load_cell_retract.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def load_class(path, name, namespace):
    """Execute the actual class without importing unrelated Linux-only IO."""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class ProfileTests(unittest.TestCase):
    def verify(self, distance, sd, speed, accel, frequency):
        segments, clocks = module.build_profile(distance, sd, speed, accel, frequency)
        reconstructed, clock = [], 0
        for iv, count, add in segments:
            self.assertTrue(1 <= count <= 4096)
            self.assertTrue(-32768 <= add <= 32767)
            for j in range(count):
                interval = iv + j*add
                self.assertGreaterEqual(interval, math.ceil(frequency*.000002))
                clock += interval
                reconstructed.append(clock)
        self.assertEqual(clocks, reconstructed)
        self.assertEqual(len(clocks), math.ceil(distance/sd))
        self.assertLessEqual(len(segments), 512)
        self.assertLessEqual(clocks[-1], frequency*.5)
        self.assertTrue(all(a < b for a,b in zip(clocks, clocks[1:])))
        # Quantization may add time, but must never advance past the ideal
        # continuous trapezoid by more than one clock tick.
        d = len(clocks)*sd
        peak = min(speed, math.sqrt(d*accel))
        ramp = peak/accel
        total = d/peak + ramp
        for i, clock in enumerate(clocks, 1):
            t = clock/frequency
            if t < ramp:
                ideal = .5*accel*t*t
            elif t <= total-ramp:
                ideal = .5*accel*ramp*ramp + peak*(t-ramp)
            else:
                ideal = d-.5*accel*max(0,total-t)**2
            self.assertLessEqual(i*sd, ideal+speed/frequency+1e-12)
        return segments, clocks

    def test_m1_profile(self):
        for frequency in (12000000, 48000000, 125000000):
            with self.subTest(frequency=frequency):
                segments, clocks = self.verify(.4, 1/3840, 10, 1000, frequency)
                self.assertEqual(len(clocks), 1536)
                self.assertLess(len(segments), 200)
                self.assertAlmostEqual(clocks[-1]/frequency, .05, delta=.0002)

    def test_random_supported_profiles(self):
        rng = random.Random(38)
        accepted = 0
        for _ in range(200):
            args = (rng.uniform(.05,.5), 1/rng.choice([400,800,1600,3840]),
                    rng.uniform(2,12), rng.uniform(300,1500), 12000000)
            try:
                self.verify(*args)
            except ValueError:
                continue  # Firmware capacity is a deliberate hard limit.
            accepted += 1
        self.assertGreater(accepted, 150)

    def test_invalid_parameters(self):
        valid = [.4,1/3840,10,1000,12000000]
        for i in range(5):
            for bad in (0,-1,float('nan'),float('inf')):
                args = valid[:]; args[i] = bad
                with self.assertRaises(ValueError): module.build_profile(*args)
        for args in ((10,1/3840,10,1000,12000000),
                     (.4,1/3840,.1,1000,12000000),
                     (.001,.000001,100,1e12,12000000)):
            with self.assertRaises(ValueError): module.build_profile(*args)


def fixture(inverted=False):
    r = module.AutomaticRetract.__new__(module.AutomaticRetract)
    r.printer = Mock()
    r.printer.command_error = RuntimeError
    r.reactor = Mock()
    r.reactor.monotonic.return_value = 0.
    r.endstop = Mock()
    r.endstop.home_wait.return_value = .9
    r.stepper = Mock()
    r.stepper.get_dir_inverted.return_value = (inverted,inverted)
    r.stepper.get_step_dist.return_value = 1/3840
    r.stepper.get_pulse_duration.return_value = (.0000002, False)
    r.stepper.mcu_to_commanded_position.side_effect = lambda pos: pos/3840 + 2.
    r.mcu = Mock()
    r.mcu.seconds_to_clock.side_effect = lambda t: int(round(t*12000000))
    r.mcu.print_time_to_clock.side_effect = r.mcu.seconds_to_clock.side_effect
    r.mcu.clock32_to_clock64.side_effect = lambda c: c
    r.mcu.clock_to_print_time.side_effect = lambda c: c/12000000
    r.mcu.is_fileoutput.return_value = False
    toolhead, kin = Mock(), Mock()
    toolhead.get_kinematics.return_value = kin
    toolhead.get_position.return_value = [10,10,2,0]
    toolhead.get_max_velocity.return_value = (250,3000)
    kin.max_z_velocity, kin.max_z_accel = 15,1000
    kin.get_status.return_value = dict(homed_axes='xyz',axis_maximum=NS(z=160))
    r.printer.lookup_object.side_effect = lambda name, default=None: (
        toolhead if name == 'toolhead' else default)
    r.oid=0; r.reason=255; r.cycle=1; r.profile_key=None; r.result=None
    r.segment_cmd=Mock(); r.arm_cmd=Mock(); r.query_cmd=Mock()
    r.cancel_cmd=Mock(); r.barrier_cmd=Mock()
    r.gcmd=Mock(); r.gcmd.error=RuntimeError
    r.params={'load_cell_retract_dist':.4,'lift_speed':10}
    r.prepare(r.gcmd,r.params)
    sign = -1 if inverted else 1
    r.state = dict(cycle=1,state=3,start=12000000,
                   end=12000000+r.clocks[-1]+4,
                   halt=sign*3840,pos=sign*(3840+len(r.clocks)))
    r.query_cmd.send.return_value=r.state
    return r


class LifecycleTests(unittest.TestCase):
    def test_position_and_direction_both_inversions(self):
        for inverted in (False,True):
            r=fixture(inverted)
            self.assertEqual(r.direction, not inverted)
            self.assertEqual(r.home_wait(2.), .9)
            self.assertAlmostEqual(r.halt_z,3.)
            self.assertAlmostEqual(r.z_at(r.ascent_start),3.)
            self.assertAlmostEqual(r.z_at(r.ascent_end),3.4)
            for i in (0,20,500,1535):
                self.assertAlmostEqual(r.z_at((r.start_clock+r.clocks[i])/12000000),
                                       3.+(i+1)/3840)
            r.stepper.note_homing_end.assert_called_once()
            r.printer.invoke_shutdown.assert_not_called()

    def test_profile_cached(self):
        r=fixture()
        r.segment_cmd.reset_mock(); r.query_cmd.reset_mock()
        r.prepare(r.gcmd,r.params)
        r.segment_cmd.send.assert_not_called()
        r.query_cmd.send.assert_not_called()
        r.prepare(r.gcmd,dict(load_cell_retract_dist=.3,lift_speed=10))
        self.assertTrue(r.segment_cmd.send.called)

    def test_failures_shutdown_without_position_sync(self):
        for change in ({'cycle':99},{'state':4},{'pos':1},{'end':0}):
            r=fixture(); r.state.update(change)
            with self.assertRaises(RuntimeError): r.home_wait(2.)
            r.cancel_cmd.send.assert_called_once_with([0])
            r.printer.invoke_shutdown.assert_called_once()
            r.stepper.note_homing_end.assert_not_called()

    def test_no_trigger_is_not_success(self):
        r=fixture(); r.endstop.home_wait.return_value=0.
        with self.assertRaises(RuntimeError): r.home_wait(2.)
        r.printer.invoke_shutdown.assert_called_once()

    def test_cancel_transport_failure_still_shuts_down(self):
        r=fixture(); r.state['state']=4
        r.cancel_cmd.send.side_effect=RuntimeError('disconnected')
        with self.assertRaises(RuntimeError): r.home_wait(2.)
        r.printer.invoke_shutdown.assert_called_once()

    def test_position_sync_failure_shuts_down(self):
        r=fixture(); r.stepper.note_homing_end.side_effect=RuntimeError('sync')
        with self.assertRaises(RuntimeError): r.home_wait(2.)
        r.printer.invoke_shutdown.assert_called_once()

    def test_active_wait_and_timeout(self):
        r=fixture()
        r.query_cmd.send.side_effect=[dict(r.state,state=2),r.state]
        r.home_wait(2.)
        r.reactor.pause.assert_called_once()
        r=fixture(); r.query_cmd.send.return_value=dict(r.state,state=2)
        r.reactor.monotonic.side_effect=[0.,2.]
        with self.assertRaises(RuntimeError): r.home_wait(2.)
        r.printer.invoke_shutdown.assert_called_once()

    def test_clock_wrap(self):
        r=fixture()
        r.state['start']=0xfffff000
        r.state['end']=(r.state['start']+r.clocks[-1]+4)&0xffffffff
        r.mcu.clock32_to_clock64.side_effect=lambda c: c+(1<<32)
        r.home_wait(2.)
        self.assertEqual(r.start_clock,0xfffff000+(1<<32))
        self.assertAlmostEqual(r.z_at(r.ascent_end),3.4)

    def test_arm_barrier_order(self):
        r=fixture(); order=[]
        r.barrier_cmd.send.side_effect=lambda args: order.append('barrier')
        r.arm_cmd.send.side_effect=lambda args: order.append(('arm',args))
        r.query_cmd.send.side_effect=lambda args: dict(cycle=2,state=1)
        def start(*args,**kwargs):
            order.append('dispatch')
            kwargs['arm_callback']()
            order.append('sensor')
            return 'completion'
        r.endstop.home_start.side_effect=start
        self.assertEqual(r.home_start(1.,0.,0,0.),'completion')
        self.assertEqual(order,['dispatch','barrier',('arm',[0,2,12000,1,255]),'sensor'])

    def test_arm_failure_shutdown(self):
        r=fixture(); r.endstop.home_start.side_effect=RuntimeError('arm')
        with self.assertRaises(RuntimeError): r.home_start(1.,0.,0,0.)
        r.printer.invoke_shutdown.assert_called_once()

    def test_headroom_homing_and_shaper_checks(self):
        r=fixture(); th=r.printer.lookup_object('toolhead'); kin=th.get_kinematics()
        kin.get_status.return_value['homed_axes']='xy'
        with self.assertRaisesRegex(RuntimeError,'home Z'): r.prepare(r.gcmd,r.params)
        kin.get_status.return_value['homed_axes']='xyz'
        th.get_position.return_value=[0,0,160,0]
        with self.assertRaisesRegex(RuntimeError,'headroom'): r.prepare(r.gcmd,r.params)
        th.get_position.return_value=[0,0,2,0]
        shaper=Mock(); axis=Mock(); axis.get_name.return_value='shaper_z'
        axis.is_enabled.return_value=True; shaper.get_shapers.return_value=[axis]
        r.printer.lookup_object.side_effect=lambda name,default=None: {
            'toolhead':th,'input_shaper':shaper}.get(name,default)
        with self.assertRaisesRegex(RuntimeError,'shaping'): r.prepare(r.gcmd,r.params)


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        path=ROOT/'klippy/extras/load_cell_probe.py'
        ns={}
        self.move_class=load_class(path,'LoadCellProbingMove',ns)
        self.tap_class=load_class(path,'TappingMove',{
            'ASCENT_DATA_WINDOW_SECONDS':.3,'FIT_MIN_POINTS':3,
            '_lookup_z_pos':lambda *a: (_ for _ in ()).throw(AssertionError('old history'))})

    def test_only_mesh_uses_automatic_retract(self):
        for command in ('BED_MESH_CALIBRATE','G28','PROBE','PROBE_ACCURACY','Z_TILT_ADJUST'):
            move=self.move_class.__new__(self.move_class)
            move._printer=Mock(); move._config_helper=Mock()
            move._param_helper=Mock()
            move._param_helper.get_probe_params.return_value={'probe_speed':4}
            move._automatic_retract=Mock(); move._mcu_trigger_analog=Mock()
            move._pause_and_tare=Mock(); move._start_collector=Mock()
            move._z_min_position=-4
            th=Mock(); th.get_position.return_value=[1,1,2,0]
            homing=Mock(); homing.probing_move.return_value=[1,1,0,0]
            move._printer.lookup_object.side_effect=lambda name: {'toolhead':th,'homing':homing}[name]
            gcmd=Mock(); gcmd.get_command.return_value=command
            move.probing_move(gcmd)
            chosen=homing.probing_move.call_args.args[0]
            self.assertIs(chosen,move._automatic_retract if command=='BED_MESH_CALIBRATE'
                          else move._mcu_trigger_analog)

    def test_ascent_fit_uses_autonomous_positions_and_window(self):
        tap=self.tap_class.__new__(self.tap_class)
        tap._load_cell_probing_move=Mock()
        tap._load_cell_probing_move._mcu.is_fileoutput.return_value=False
        tap._printer=Mock(); tap._printer.command_error=RuntimeError
        tap._best_fit=Mock(); tap._best_fit.find_best_fit.return_value=(3.1,3,3,100)
        auto=NS(ascent_end=1.05,z_at=lambda t:3.+(t-1.)*10)
        samples=[(.9,0)]+[(1.+i*.005,i) for i in range(11)]+[(1.06,999)]
        self.assertEqual(tap._analyze_ascent(Mock(),samples,1.,Mock(),3.,auto),3.1)
        data=tap._best_fit.find_best_fit.call_args.args[0]
        self.assertEqual(len(data),11)
        self.assertAlmostEqual(data[-1][1],3.5)

    def test_real_fit_with_reconstructed_step_positions(self):
        sys.path.insert(0,str(ROOT/'klippy'))
        spec=importlib.util.spec_from_file_location('mathutil',ROOT/'klippy/mathutil.py')
        mathutil=importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mathutil)
        finally:
            sys.path.pop(0)
        fitter=load_class(ROOT/'klippy/extras/load_cell_probe.py','LCBestFit',
                          {'mathutil':mathutil,'sys':sys})(Mock())
        r=fixture(); r.home_wait(2.)
        data=[]; contact=3.08
        for i in range(int((r.ascent_end-r.ascent_start)*2000)):
            z=r.z_at(r.ascent_start+i/2000)
            data.append((1000*max(contact-z,0),z))
        actual,below,above,slope=fitter.find_best_fit(data)
        self.assertAlmostEqual(actual,contact,delta=.0001)
        self.assertGreaterEqual(min(below,above),3)

    def test_actual_homing_keeps_trigger_and_lifted_position_separate(self):
        import logging
        path=ROOT/'klippy/extras/homing.py'
        ns={'math':math,'logging':logging,'HOMING_START_DELAY':.001,
            'ENDSTOP_SAMPLE_TIME':.000015,'ENDSTOP_SAMPLE_COUNT':4,
            'multi_complete':lambda printer,completions:completions[0]}
        load_class(path,'StepperPosition',ns)
        cls=load_class(path,'HomingMove',ns)
        r=fixture()
        stepper=r.stepper
        stepper.get_name.return_value='stepper_z'
        stepper.get_commanded_position.return_value=4.
        stepper.get_mcu_position.return_value=7680
        stepper.get_past_mcu_position.return_value=3840
        stepper.calc_position_from_coord.side_effect=lambda p:p[2]
        # Mirror the host query's position update, retaining the original
        # coordinate mapping for verify_no_probe_skew after set_position.
        def synchronize():
            stepper.get_mcu_position.side_effect=lambda pos=None: (
                7680 if pos is not None else 5376)
        stepper.note_homing_end.side_effect=synchronize
        r.endstop.get_steppers.return_value=[stepper]
        r.endstop.home_start.return_value=object()
        th=r.printer.lookup_object('toolhead'); kin=th.get_kinematics()
        kin.get_steppers.return_value=[stepper]
        kin.calc_position.side_effect=lambda d:[1.,1.,d['stepper_z']]
        th.get_position.return_value=[1.,1.,4.,0.]
        th.get_last_move_time.return_value=1.
        result=cls(r.printer,[(r,'probe')],th).homing_move(
            [1.,1.,-4.,0.],4.,probe_pos=True)
        self.assertAlmostEqual(result[2],3.)
        self.assertAlmostEqual(th.set_position.call_args.args[0][2],3.4)


if __name__ == '__main__': unittest.main()
