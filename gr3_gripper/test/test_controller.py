import json
import os
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from gr3_gripper.controller import ACTUATOR, CoreLayout, GripperController
from gr3_gripper.protocol import BACKEND, GripperSession

FIXTURE = Path(__file__).resolve().parents[1] / "models/smoke_fixture.xml"


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(FIXTURE))
        self.data = mujoco.MjData(self.model)
        self.gripper = GripperController(self.model)

    def test_rate_mapping_and_physics(self):
        self.gripper.set_target(1)
        self.gripper.update(self.data)
        for _ in range(500):
            mujoco.mj_step(self.model, self.data)
            self.gripper.update(self.data)
        self.assertAlmostEqual(self.gripper.current, 0.5, places=6)
        self.assertAlmostEqual(self.data.ctrl[self.gripper.aid], 127.5, places=5)
        for _ in range(700):
            mujoco.mj_step(self.model, self.data)
            self.gripper.update(self.data)
        self.assertEqual(self.gripper.current, 1.0)
        self.assertGreater(self.data.qpos[0], 0.6)
        self.assertTrue(np.isfinite(self.data.qpos).all())
        self.assertLessEqual(abs(self.data.actuator_force[self.gripper.aid]), 5.000001)

    def test_open_from_closed_is_rate_limited(self):
        self.gripper.set_target(1)
        self.gripper.update(self.data)
        self.data.time = 2
        self.gripper.update(self.data)
        self.gripper.set_target(0)
        self.data.time = 2.5
        self.gripper.update(self.data)
        self.assertEqual(self.gripper.current, 0.75)

    def test_invalid_inputs_do_not_change_target(self):
        self.gripper.set_target(0.4)
        for value in [True, -1, 2, float("nan"), float("inf"), None]:
            with self.assertRaises((ValueError, TypeError)):
                self.gripper.set_target(value)
        self.assertEqual(self.gripper.target, 0.4)

    def test_reset_preserves_original_open_behavior(self):
        self.gripper.set_target(1)
        self.gripper.update(self.data)
        self.data.time = 2
        self.gripper.update(self.data)
        self.data.time = 0
        self.gripper.update(self.data)
        self.assertEqual(self.gripper.current, 0)
        self.assertEqual(self.gripper.target, 0)

    def test_nonfinite_time_rejected(self):
        self.data.time = float("nan")
        with self.assertRaises(ValueError):
            self.gripper.update(self.data)

    def test_actuator_range_and_force_limits_checked(self):
        self.model.actuator_ctrlrange[0, 1] = 1
        with self.assertRaises(ValueError):
            GripperController(self.model)
        self.model.actuator_ctrlrange[0, 1] = 255
        self.model.actuator_forcelimited[0] = 0
        with self.assertRaises(ValueError):
            GripperController(self.model)

    def test_missing_actuator_cannot_accept_commands(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
        gripper = GripperController(model)
        self.assertFalse(gripper.enabled)
        with self.assertRaises(ValueError):
            gripper.set_target(1)

    def test_only_gripper_actuator_is_written(self):
        xml = FIXTURE.read_text()
        bodies = ''.join('<body><joint name="core_%d"/><geom size="0.01" mass="0.1"/></body>' % i
                         for i in range(31))
        motors = ''.join('<motor joint="core_%d"/>' % i for i in range(31))
        xml = xml.replace('<worldbody>', '<worldbody>' + bodies).replace('<actuator>', '<actuator>' + motors)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        data.ctrl[:31] = np.arange(31) + 0.25
        expected = data.ctrl[:31].copy()
        gripper = GripperController(model)
        gripper.set_target(1)
        gripper.update(data)
        data.time = 1
        gripper.update(data)
        np.testing.assert_array_equal(data.ctrl[:31], expected)
        self.assertEqual(data.ctrl[31], 127.5)

    def test_core_apply_preserves_gripper_control(self):
        # Exercise the existing Core output boundary without a full robot scene.
        core = CoreLayout.__new__(CoreLayout)
        core.actuators = list(range(31))
        core.qadr = np.arange(31)
        core.vadr = np.arange(31, 62)
        data = SimpleNamespace(ctrl=np.ones(32) * 123, sensordata=np.zeros(62))
        command = SimpleNamespace(motorNum=31, started=1, qCmd=np.ones(31),
                                  qDotCmd=np.zeros(31), torqCmd=np.zeros(31),
                                  q_kp=np.ones(31), q_kd=np.zeros(31))
        self.assertTrue(core.apply(data, command))
        self.assertEqual(data.ctrl[31], 123)
        command.qCmd[0] = float("nan")
        self.assertFalse(core.apply(data, command))
        self.assertEqual(data.ctrl[31], 123)

    def test_old_file_not_replayed_and_nonobject_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "command.json"
            path.write_text(json.dumps(dict(sequence=1, closure=1)))
            controller = GripperController(self.model, command_file=str(path))
            controller.poll()
            self.assertEqual(controller.target, 0)
            path.write_text('[]')
            controller.next_poll = 0
            controller.poll()
            self.assertIn("object", controller.last_error)


class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(FIXTURE))
        self.data = mujoco.MjData(self.model)
        self.gripper = GripperController(self.model)
        self.now = 100.0
        self.session = GripperSession(self.gripper, clock=lambda: self.now)
        self.command = dict(version=1, id="one", server_id=self.session.server_id,
                            backend=BACKEND, closure=1.0, expires_at=110)

    def send(self, **kwargs):
        return self.session.request(json.dumps(dict(self.command, **kwargs)))

    def test_ramp_completion_never_claims_grasp(self):
        self.assertEqual(self.send()["state"], "ACCEPTED")
        self.gripper.update(self.data)
        self.assertEqual(self.session.sample(self.data)["state"], "RAMPING")
        self.data.time = 2
        self.gripper.update(self.data)
        result = self.session.sample(self.data)
        self.assertEqual(result["state"], "RAMP_COMPLETE")
        self.assertIsNone(result["robot_task_success"])
        self.assertEqual(result["observation"]["driver_positions"], [0, 0])

    def test_duplicate_and_busy(self):
        self.send()
        self.assertTrue(self.send()["duplicate"])
        self.assertEqual(self.send(id="two")["state"], "REJECTED")
        self.assertEqual(self.send(closure=0)["state"], "REJECTED")

    def test_invalid_commands_are_rejected(self):
        cases = [dict(closure=True), dict(closure="1"), dict(closure=float("nan")),
                 dict(closure=2), dict(expires_at=99), dict(expires_at=131),
                 dict(server_id="old"), dict(backend="aurora"), dict(version=True),
                 dict(id=[]), dict(extra="ignored?")]
        for case in cases:
            with self.subTest(case=case):
                self.assertEqual(self.send(**case)["state"], "REJECTED")
        for raw in ['[]', 'null', '{', '"text"', 'x' * 4097]:
            self.assertEqual(self.session.request(raw)["state"], "REJECTED")
        self.assertEqual(self.gripper.target, 0)

    def test_timeout_holds_current_command(self):
        self.send()
        self.gripper.update(self.data)
        self.data.time = 0.5
        self.gripper.update(self.data)
        self.now = 111
        result = self.session.sample(self.data)
        self.assertEqual(result["state"], "TIMEOUT")
        self.assertEqual(self.gripper.target, 0.25)

    def test_reset_rotates_session(self):
        self.send()
        old = self.session.server_id
        self.assertEqual(self.session.reset()["state"], "RESET")
        self.assertNotEqual(self.session.server_id, old)
        self.assertEqual(self.send()["state"], "REJECTED")

    def test_command_file_and_ros_cannot_both_own_gripper(self):
        with tempfile.TemporaryDirectory() as tmp:
            controller = GripperController(self.model, command_file=str(Path(tmp) / "unused.json"))
            with self.assertRaises(ValueError):
                GripperSession(controller)

    def test_nonfinite_feedback_is_not_serialized(self):
        self.data.qpos[0] = float("nan")
        with self.assertRaises(ValueError):
            self.session.sample(self.data)


@unittest.skipUnless(os.getenv("GR3_TEST_MODEL"), "Set GR3_TEST_MODEL to an existing integrated XML/MJB")
class ExistingModelTest(unittest.TestCase):
    def test_existing_model_control_isolated(self):
        path = os.environ["GR3_TEST_MODEL"]
        loader = mujoco.MjModel.from_binary_path if path.endswith(".mjb") else mujoco.MjModel.from_xml_path
        model = loader(path)
        data = mujoco.MjData(model)
        gripper = GripperController(model)
        self.assertTrue(gripper.enabled)
        data.ctrl[:] = np.arange(model.nu) + 0.123
        before = data.ctrl.copy()
        gripper.set_target(0.6)
        gripper.update(data)
        data.time = 2
        gripper.update(data)
        mask = np.arange(model.nu) != gripper.aid
        np.testing.assert_array_equal(data.ctrl[mask], before[mask])
        self.assertAlmostEqual(data.ctrl[gripper.aid], 153.0)


if __name__ == "__main__":
    unittest.main()
