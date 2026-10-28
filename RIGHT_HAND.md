# GR3 right-hand Robotiq 2F-85 integration

This change adds ROS2 open/close control to the right hand of GR3 V233.
It is based on team `dev` commit `d5f34fa2a6c5844b3da49dba7305a4e4e3b169b4`.
The left dummy hand and all original right wrist joints remain unchanged.
The gripper mount is rigidly attached to `right_end_effector_link`; it follows
the arm and wrist and has no additional mount rotation joint.

## Files and model source

- `simulation/sensors/hand_controller.py`: live-model control, ROS2 protocol,
  cleanup and body-channel validation.
- `simulation/sensors/viewer.py`: binding, physics-loop calls and body-channel
  compatibility; the team's camera and lidar code is retained.
- `simulation/sensors/gr3v2_3_3.xml`: replaces only the right dummy hand.
- `simulation/grippers/robotiq_2f85/`: MJCF, meshes, reproducible conversion,
  pinned Kinova Xacro/DAE sources, provenance and BSD license.
- `tests/test_hand.py`: standalone physics and protocol checks.

Kinova ros_kortex is the model source. Menagerie was a reference for MJCF
mechanisms, not the source of this model's meshes or inertias. See
`simulation/grippers/robotiq_2f85/PROVENANCE.md` and `LICENSE` in that directory.
The retired `gr3_gripper` package is not used.

## ROS2 commands

Use the same ROS2 domain as the viewer (the tested deployment uses
`ROS_DOMAIN_ID=6`). Aurora's `domain_id=123` is separate and unchanged.
Source ROS2 Humble before using these commands.

In a separate terminal, listen before sending commands:

```bash
ros2 topic echo /hand_status std_msgs/msg/String
```

Close, then wait for the terminal status before sending open:

```bash
ros2 topic pub --once /hand_command std_msgs/msg/String \
  '{"data":"{\"id\":101,\"action\":\"close\"}"}'
```

```bash
ros2 topic pub --once /hand_command std_msgs/msg/String \
  '{"data":"{\"id\":102,\"action\":\"open\"}"}'
```

Feedback is a JSON string with the same `id` and `state`: `RUNNING`, then
`SUCCESS` or `FAIL` (with a `reason` where applicable). IDs identify requests;
101 is not an action number. Use a new ID for a new action. The latest identical
request is deduplicated; a new request while busy is rejected, not queued.

The controller holds its initial opening until commanded. The joint target is
0 rad for open and 0.8 rad for close, ramped at 0.4 rad per simulation second.
Completion requires measured finger positions and velocities to settle.
The wall-clock timeout is 30 seconds and includes pause time. `SUCCESS` means
the opening target was reached, not that an object was grasped. Object contact
can prevent reaching the close target and produce a timeout.

## Viewer integration and compatibility

`hand.bind(m, d)` uses the viewer's current model and data. The controller does
not load a separate simulation, step physics or write joint positions.
`hand.update()` runs on the physics thread after body control and before
physics stepping; paused calls process messages/timeouts without ramping.
The wrapper releases the ROS node on normal and exceptional loop exit, and
binding a new model releases the previous node.

The gripper adds one actuator after the original 31 body motors (ID 31,
total `nu=32`). Body control and state publishing remain 31-dimensional, so
they cannot overwrite the hand command. No hand sensors are appended.
A handless model disables the controller without importing ROS. Initialization
errors are reported; an unexpected body/sensor layout raises a clear error.

The viewer patch is +18/-10 lines, the expanded scope accepted for preserving
the body channels. Existing camera/lidar and model-reload behavior is retained;
this does not certify old models' compatibility with those pre-existing modules.
GR3 V211 GUI startup and live model-reload regression remain unverified.

## Deployment layout

Checking out this branch does not update a running SDK viewer or installed
Python package. First inspect the actual running command and back up any files
being replaced. Stop the affected simulation before replacing its files.

For a standard SDK layout:

| Repository source | SDK destination |
| --- | --- |
| `simulation/sensors/hand_controller.py` | beside the actual `sim/mujoco/viewer.py` |
| `simulation/sensors/viewer.py` | actual `sim/mujoco/viewer.py`, after reviewing its diff |
| `simulation/sensors/gr3v2_3_3.xml` | `GR3_V233/mjcf/gr3v2_3_3.xml` |
| entire `simulation/grippers/robotiq_2f85/` | `GR3_V233/meshes/robotiq_2f85/` |

Retain vendor `main.xml`, actuator/sensor XML and original GR3 meshes. The
robot XML includes `../meshes/robotiq_2f85/{definitions,body}.xml`; copying only
the robot XML is insufficient. Vendor GR3 resources are supplied by the SDK,
not redistributed here. The gripper resources are all included in this repo.

The repository viewer also imports the team's `rgbd_camera.py` and `lidar_2d.py`;
deploy their matching existing dependencies if using that viewer. The user's
successful Docker demo used a separate SDK test viewer without camera/lidar.
That demo is evidence for the gripper integration, not end-to-end validation
of the combined repository camera/lidar viewer. The installed `start_gr3`
package and Aurora control flow require no changes for open/close control.

## Validation and limits

Re-run from the repository root with Python 3.10, NumPy and MuJoCo installed:

```bash
python3 -m unittest discover -s tests -p test_hand.py -v
```

The 11 tests cover physical opening/closing, slow-simulation timeout allowance,
measured arrival, pause/reset/nonfinite feedback, handless mode, body-channel
isolation/layout checks, original hand/wrist preservation, protocol handling
and lifecycle cleanup. ROS message tests use a stub; they do not test DDS.
Randomized test setup uses seed 42. No GPU is required for these tests.

User-reported Ubuntu/Docker results (Python 3.10.12, MuJoCo 2.3.6): standalone
and full V233 model load succeeded with 31 body / 32 total actuators; close/open
requests returned `SUCCESS`, with reported durations around 7 seconds each.
The team accepted the displayed gripper demo on 2026-10-28. These are user
reports, not direct remote observations by the code author.

This branch contains no bowl mass changes, kitchen solver changes, navigation,
arm planning, grasp/contact-success logic or cabinet placement implementation.
