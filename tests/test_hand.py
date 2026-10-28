"""Physics and protocol unit checks. These do NOT simulate a real ROS middleware."""
from pathlib import Path
import ast
import importlib.util
import json
import sys
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'simulation/sensors'))
from hand_controller import ACTUATOR, HandController, HandDrive, body_motor_count, with_hand_controller
MODEL = ROOT/'simulation/grippers/robotiq_2f85/standalone.xml'


class HandTests(unittest.TestCase):
    def setUp(self):
        np.random.seed(42)
        self.m = mujoco.MjModel.from_xml_path(str(MODEL))
        self.d = mujoco.MjData(self.m)

    def test_close_open_physics_and_feedback(self):
        drive = HandDrive(self.m, self.d)
        for action, target in [('close', .8), ('open', 0)]:
            drive.start(action)
            outcome = None
            for _ in range(6000):
                outcome = drive.update()
                if outcome:
                    break
                mujoco.mj_step(self.m, self.d)
            self.assertEqual(outcome, 'reached')
            np.testing.assert_allclose(self.d.qpos[drive.qadr], drive.signs*target, atol=.02)
            self.assertTrue(np.isfinite(self.d.qpos).all())

    def test_ramp_alone_is_not_success(self):
        drive = HandDrive(self.m, self.d)
        drive.start('close')
        self.d.time = 3.0
        self.assertIsNone(drive.update())
        self.assertEqual(drive.commanded, .8)
        self.assertTrue(drive.active)
        self.assertEqual(self.d.qpos[0], 0.0)

    def test_close_open_with_slow_simulation(self):
        # Five wall-clock milliseconds per 1 ms physics step, as on a slow VM.
        clock = [0.0]
        with patch('hand_controller.time.monotonic', side_effect=lambda: clock[0]):
            drive = HandDrive(self.m, self.d)
            for action in ('close', 'open'):
                drive.start(action)
                started = clock[0]
                outcome = None
                for _ in range(6000):
                    outcome = drive.update()
                    if outcome:
                        break
                    mujoco.mj_step(self.m, self.d)
                    clock[0] += .005
                self.assertEqual(outcome, 'reached')
                self.assertGreater(clock[0]-started, 8)

    def test_paused_timeout_and_reset(self):
        drive = HandDrive(self.m, self.d)
        drive.start('close')
        for _ in range(10):
            self.assertIsNone(drive.update(running=False))
        self.assertEqual(drive.commanded, 0.0)
        drive.deadline = time.monotonic()-1
        self.assertEqual(drive.update(False), 'timeout')
        drive.start('close')
        self.d.time = 1
        drive.update()
        mujoco.mj_resetData(self.m, self.d)
        self.assertEqual(drive.update(), 'reset')
        self.assertFalse(drive.active)

    def test_nonfinite_feedback(self):
        drive = HandDrive(self.m, self.d)
        self.d.qpos[0] = np.nan
        with self.assertRaises(ValueError):
            drive.update()

    def test_no_hand_does_not_import_ros(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
        with HandController() as hand:
            hand.bind(model, mujoco.MjData(model))
            self.assertIsNone(hand.node)
            hand.update()
        self.assertEqual(body_motor_count(model), 0)

    def test_body_channels_are_isolated(self):
        xml = ET.parse(MODEL).getroot()
        xml.find('compiler').set('meshdir', str(MODEL.parent.parent))
        xml.find('include').set('file', str(MODEL.parent/'definitions.xml'))
        xml.find('worldbody/include').set('file', str(MODEL.parent/'body.xml'))
        actuators = ET.Element('actuator')
        xml.insert(2, actuators)  # Must precede included hand actuator.
        sensors = ET.SubElement(xml, 'sensor')
        world = xml.find('worldbody')
        for i in range(31):
            body = ET.SubElement(world, 'body', pos=f'{i} 0 1')
            ET.SubElement(body, 'joint', name=f'body_{i}')
            ET.SubElement(body, 'geom', type='sphere', size='.01', mass='.1')
            ET.SubElement(actuators, 'motor', joint=f'body_{i}')
        for kind in ('jointpos', 'jointvel'):
            for i in range(31):
                ET.SubElement(sensors, kind, joint=f'body_{i}')
        model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))
        data = mujoco.MjData(model)
        self.assertEqual(body_motor_count(model), 31)
        drive = HandDrive(model, data)
        self.assertEqual(drive.aid, 31)
        expected = np.arange(31, dtype=float)+.123
        data.ctrl[:31] = expected
        drive.start('close')
        data.time = 1
        drive.update()
        np.testing.assert_array_equal(data.ctrl[:31], expected)
        self.assertAlmostEqual(data.ctrl[31], .4)
        # Execute the actual proposed body-control function without a GUI/LCM session.
        tree = ast.parse((ROOT/'simulation/sensors/viewer.py').read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_control')
        scope = dict(np=np, body_motor_count=body_motor_count, lcm_sub_msg=None)
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<viewer apply_control>', 'exec'), scope)
        scope['apply_control'](model, data)
        np.testing.assert_array_equal(data.ctrl[:31], np.zeros(31))
        self.assertAlmostEqual(data.ctrl[31], .4)
        scope['lcm_sub_msg'] = SimpleNamespace(started=1, q_kp=np.ones(31),
            q_kd=np.ones(31), qCmd=np.ones(31), qDotCmd=np.zeros(31), torqCmd=np.zeros(31))
        scope['apply_control'](model, data)
        np.testing.assert_array_equal(data.ctrl[:31], np.ones(31))
        self.assertAlmostEqual(data.ctrl[31], .4)

    def test_body_layout_change_is_detected(self):
        # A standalone hand is valid for HandDrive, but cannot be treated as GR3's body.
        with self.assertRaises(ValueError):
            body_motor_count(self.m)

    def test_left_hand_and_wrist_are_preserved(self):
        source = ET.parse(ROOT/'simulation/sensors/gr3v2_3_3.xml').getroot()
        wrists = [j for j in source.findall('.//joint') if (j.get('name') or '').startswith('right_wrist_')]
        self.assertEqual(len(wrists), 3)
        self.assertIsNotNone(source.find('.//body[@name="left_dummy_hand_link"]'))
        self.assertIsNone(source.find('.//body[@name="dummy_right_hand_link"]'))
        endpoint = source.find('.//body[@name="right_end_effector_link"]')
        self.assertEqual(endpoint.find('include').get('file'), '../meshes/robotiq_2f85/body.xml')


class ProtocolTests(unittest.TestCase):
    def test_loop_wrapper_closes_on_normal_and_exception_exit(self):
        for fail in (False, True):
            controller = HandController()
            with patch.object(controller, 'close') as close:
                with patch('hand_controller.HandController', return_value=controller):
                    @with_hand_controller
                    def loop(value, *, hand):
                        self.assertIs(hand, controller)
                        if fail:
                            raise RuntimeError('loop failed')
                        return value
                    if fail:
                        with self.assertRaisesRegex(RuntimeError, 'loop failed'):
                            loop(42)
                    else:
                        self.assertEqual(loop(42), 42)
                close.assert_called_once_with()

    def test_simple_commands_and_duplicate(self):
        class Recorder(HandController):
            def _publish(self, command_id, state, reason=None):
                result = dict(id=command_id, state=state)
                if reason:
                    result['reason'] = reason
                self.messages.append(result)
                return result
        hand = Recorder()
        hand.messages = []
        for payload in [dict(id=1, action='close'), dict(id=1, action='close'), dict(id=2, action='open')]:
            hand._receive(SimpleNamespace(data=json.dumps(payload)))
        self.assertEqual([m['state'] for m in hand.messages], ['RUNNING','RUNNING','FAIL'])
        self.assertEqual(hand.pending, dict(id=1, action='close'))
        for raw in ['[]', '{', '{"id":true,"action":"open"}', '{"id":3,"action":"grasp"}']:
            hand._receive(SimpleNamespace(data=raw))
            self.assertEqual(hand.messages[-1]['state'], 'FAIL')


if __name__ == '__main__':
    unittest.main()
