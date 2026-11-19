# Feather Quest teleoperation

Drive a Feather robot with Meta Quest controllers and record what it does. Each controller moves one hand, and every demonstration is saved with the cameras' video. The headset is used one of two ways:

- **Worn**: you wear it and see the robot's stereo camera. The robot's neck follows your head.
- **Table**: it sits on a table facing you, and you watch the robot directly. The joysticks move the neck.

The controller logic follows [DROID](https://droid-dataset.github.io)'s. This is research code, developed on a Feather robot with two 7-joint arms, the only hardware it has run on.

## What you need

- A Feather robot with FeatherSDK and its URDF. The SDK must have the features listed in [feathersdk_new/](feathersdk_new/README.md), which are not in a released SDK yet.
- A Meta Quest with its controllers.
- Worn: nothing on the headset; it opens a page in its browser. The headset must reach the robot over the network.
- Table: the headset in developer mode with the app from [rail-berkeley/oculus_reader](https://github.com/rail-berkeley/oculus_reader) (`adb install -r <its APK>`), and a computer with `adb` for its USB cable: the robot's own, or a laptop on the same network.
- Jupyter on the robot's computer.

## Install

On the robot's computer, in the SDK's Python environment (and on the laptop, if you use one):

```bash
git clone <this repository> && cd feather-teleop
pip install -e ".[robot]"    # on the laptop: pip install -e .
```

Install it editable (`-e`): the package reads `config/quest.yaml` from the checkout and saves demonstrations there.

## Run

Set `headset` in `config/quest.yaml` to `worn` or `table`.

**Worn.** The headset's browser only shows a page in the headset over https, so make a certificate once, in the repository:

```bash
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout headset_key.pem -out headset_cert.pem -subj /CN=feather
```

Run the notebook, then open `https://<robot address>:8443` in the headset's browser, accept the certificate warning and press **Enter**. `http://<robot address>:8080` leads there too.

The video comes on its own port (`worn.video_port`, 8444). If the page says it has no link to it, open `https://<robot address>:8444/` once and accept the certificate there too. `worn.debug` adds a readout in the lower left of the view: frame rates, round trip, tracking, links, the robot loop's rate and current faults.

**Table.** Plug the headset into the laptop, put it on once to accept **Allow USB debugging**, and set it down facing you. Check it, then stream it to the robot:

```bash
python -m feather_teleop.headsets.table                 # prints the controllers and buttons
python -m feather_teleop.headsets.table --to <robot address>
```

The stream is plain UDP on port 8799, so use a trusted network. With the headset plugged into the robot's computer instead, set `table.source: usb` and skip the stream.

**Then** run `notebooks/quest_teleop.ipynb` on the robot, top to bottom. It starts the robot, moves the arms to their start posture and runs the demonstration loop.

The headset's cameras must see the controllers. When they lose one, its buttons still arrive but that arm holds.

## Controls

| Control | Does |
|---|---|
| **A** | starts a demonstration: recording starts, the torso rises, the arms go to their start posture and then follow. **A** again ends it as a success |
| **B** | ends it as a failure, or stops the move to the start posture |
| **Grip**, held | that controller's arm follows; released, it stays. Re-gripping never makes it jump |
| Grip pressed twice | table: that arm returns to its start posture (a press on the way stops it). Worn: shows or hides that arm's wrist camera |
| Index trigger | closes that arm's gripper in proportion, while the grip is held |
| Left joystick | drives the base |
| Joystick clicked | locks that hand's position, so only its orientation follows; again to unlock |
| Joystick held 0.6 s | resets "forward" |

| | Table | Worn |
|---|---|---|
| Turning the base | **X** / **Y**: left / right | right joystick, sideways |
| Neck | right joystick | follows your head |
| Torso | follows the hands: rises when a gripped hand is more than 10 cm above its shoulder, sinks when more than 70 cm below | **Y** / **X**: up / down |
| "Forward" | where a controller points at its first grip | where you face when the demonstration starts |
| Gripper orientation | turns as much as the controller turns | points where the controller points |

## Safety

- An arm moves only while its grip is held, or on its way to its start posture. A stream that goes quiet for 0.2 s releases the grip. The base, neck and torso respond whenever a demonstration is running.
- The SDK's collision check refuses a target that would take an arm into the robot's body; the arm stops on its last target and the notebook says why.
- Joint limits, a speed scale, a floor height and limits on reach apply before a target is sent. The fingertips may reach the floor, but the last 15 cm down are slowed to at most 10 cm/s.
- The SDK's follow mode moves each arm toward its newest target within speed, acceleration and jerk limits.

The notebook's `ready()` and `home()` moves are **not** collision checked. Clear the space around the arms first.

## Settings

`config/quest.yaml` holds what you set, with a comment on each value. Start with `headset` and the `robot` section (your robot's SDK config version, its URDF and grippers), then check the per-robot values: `tool_offset`, `body_boxes`, `start_posture` and the neck's signs.

`arm.elbow_follow` (worn) makes the robot's elbow swing as yours does. Your elbow is estimated from the headset and controller, so at each demonstration's first grip hold your arm as the start posture: upper arm hanging, forearm forward.

Tuning is in the code, as the defaults of each settings class: `ControllerSettings` in `control/controller.py`, `ArmSettings` in `control/follower.py`, each part's settings in `parts/`, and the solver's and guards' (`IKSettings`, `GuardSettings`) in FeatherSDK's `arm_kinematics`.

## What gets saved

Each demonstration is a folder under `episodes/`:

- `episode.yaml`: `task`, `outcome` (`success`, `failure` or `interrupted`), `parts`, `start`, `end`, and each arm's joint names (`joints`).
- `samples.jsonl`: one JSON object per tick.

  | Key | Content |
  |---|---|
  | `t`, `age` | the tick's time (s since the epoch) and the age of the controller sample it used (s) |
  | `buttons` | the controllers' buttons, or `null` while the stream was stale |
  | `arms.<side>.command`, `.measured` | joint angles (rad, motor frame, in `joints` order) |
  | `arms.<side>.hand_command`, `.hand_measured`, `.target` | tool point positions (m, base frame); `target` is where the controller asks it to be, `null` without the grip |
  | `arms.<side>.following`, `.gripper`, `.refused`, `.held_back`, `.position_locked` | grip held; gripper closure (0 open, 1 closed); why the SDK refused a target; joints that stopped following (blocked); position lock |
  | `base` | `[vx, vy, yaw rate]` (m/s, rad/s) |
  | `neck` | target angle per motor (rad) |
  | `head` | worn: where your head looks, `[up, left]` (rad), or `null` while it is not tracked |
  | `torso` | `velocity` (mm/s) and `height` (mm) |

- `<camera>.mp4` and `<camera>.jsonl`: each camera's video, converted a few seconds after the demonstration ends (the `.h264` is kept if that fails), and a line per frame with `frame`, `seq`, `camera_ns` and `host_ns`.

## Not yet run on a robot

These pass their tests on FeatherSDK's test stand but have not run on hardware:

- the arms' motion through the SDK's follow mode;
- worn: the neck levelled by the stereo camera's accelerometer, the worn base and torso buttons, the wrist cameras in the headset, the video's own process and port, the debug readout, and the elbow following yours (`arm.elbow_follow`, off by default);
- the hold's bubble, the slowed approach to the floor, the gripper easing off on a held object, and the sag correction made only at rest.

## Code

```
feather_teleop/
  teleop.py               the session the notebook uses
  loop.py, episode.py     the A / A / B loop, and the record it writes
  config.py               quest.yaml, and the SDK config built from it
  headsets/               what both headsets hand the loop
    table.py              the table headset over adb, and its stream from a laptop
    worn.py, web/         the worn headset: the page the robot serves, and the video sent to it
    video_relay.py        the worn headset's video, in a process of its own
  control/
    controller.py         one controller -> a velocity command for one hand (DROID's logic)
    follower.py           one arm following one controller, through the SDK's solver and follow mode
    maths.py              rotation and filter helpers
  parts/                  what a demonstration drives: arms, base, neck, torso, cameras, display, and the joysticks they read
```

`meta_quest_teleop/` is an earlier, separate teleoperation with its own headset app. Nothing here depends on it.

## License

Apache-2.0. [NOTICE](NOTICE) credits DROID, whose controller logic and solver are ported here, and oculus_reader, whose headset app this package reads.
