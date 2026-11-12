# h1_2_inspire_description

A self-contained ROS 2 description package for the **Unitree H1-2** humanoid robot equipped with **Inspire RH56DFTP** five-finger hands. All meshes are bundled inside the package — no external dependencies on other description packages are required.

## Supported ROS 2

Tested with **ROS 2 Humble** on Ubuntu 22.04.

## Contents

| File | Description |
|---|---|
| `urdf/h1_2_body.urdf.xacro` | H1-2 body without hands (27 DOF) |
| `urdf/inspire_hand_left.urdf.xacro` | Left Inspire hand fragment (12 joints) |
| `urdf/inspire_hand_right.urdf.xacro` | Right Inspire hand fragment (12 joints) |
| `urdf/h1_2_with_inspire_hands.urdf.xacro` | Full robot — body + both hands (58 joints total) |
| `urdf/inspire_hand_left_standalone.urdf.xacro` | Left hand with world link for isolated visualization |
| `urdf/inspire_hand_right_standalone.urdf.xacro` | Right hand with world link for isolated visualization |
| `urdf/h1_2_with_RH56DFTP_hands.urdf` | **Flat URDF for Pinocchio and any plain-URDF consumer** (hands coupled by `<mimic>`) |
| `urdf/h1_2_with_RH56DFTP_hands_software_coupling.urdf` | **Same flat URDF for the software coupling**: no `<mimic>`, 24 independent hand joints |
| `urdf/h1_2_handless.urdf` | **Flat URDF, body only (27 DOF, 66.984 kg)** |
| `urdf/h1_2_hands_isaac.urdf` | **Flat URDF pre-processed for Isaac Sim 5.1** |
| `config/hand_coupling.yaml` | Software coupling: joint angle vs. command curves, one per joint (generated) |
| `config/hand_coupling_measurements.yaml` | The photo measurements those curves are fitted to |
| `scripts/hand_coupling.py` | Python module and CLI: 6 hand commands ↔ 12 joint angles |
| `scripts/hand_coupling_node.py` | ROS 2 node publishing the 12 joints of each hand |
| `tools/fit_hand_coupling.py` | Fits `config/hand_coupling.yaml` to the measurements |

### Mesh directories

```
meshes/
├── h1_2/        ← Unitree H1-2 body meshes (28 STL)
├── hand_left/   ← Inspire RH56DFTP left hand meshes (13 STL), the collision geometry
│   └── visual/  ← the same meshes split by colour (32 STL), the visuals
└── hand_right/  ← Inspire RH56DFTP right hand meshes (13 STL), the collision geometry
    └── visual/  ← the same meshes split by colour (32 STL), the visuals
```

## Kinematics summary

### H1-2 body (27 DOF)
- 6 DOF per leg × 2 (hip yaw/pitch/roll, knee, ankle pitch/roll)
- 7 DOF per arm × 2 (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
- 1 DOF torso (yaw)

### Inspire RH56DFTP hands (12 joints each)
Each hand has 6 actuated joints and 6 coupled ones (`*_2`, `*_thumb_2`, `*_thumb_3`).
How the coupled joints follow the actuated ones is chosen with the xacro argument
`coupling`: `mimic` (default) uses `<mimic>` tags, a linear approximation;
`software` drops them and computes every joint from the curves measured on the
real hand. See **Software coupling** below. Everything in this section up to that
heading describes `coupling:=mimic`.

Both hands use the **same joint names** (`*_thumb_swing`, `*_thumb_1..3`).

**`q = 0` is the hand fully open (`ANGLE_SET = 1000`) and the upper limit is fully
closed (`ANGLE_SET = 0`), on all 12 joints of each hand.** Every joint closes as
`q` grows.

**Zeros and upper limits — from photos of the real hand.** Photos of the hand at
nine `ANGLE_SET` values per DOF (1000, 875, … 0;
`inspire_hand_interface/Caracterizacion/imagenes/angulos limite/*_positions`) were
compared with this model rendered in MuJoCo through a virtual camera fitted to
each photo, sweeping the joint values until the rendered silhouette matched the
photographed one (`config/hand_coupling_measurements.yaml`). Every joint origin
is rotated so that `q = 0` is where the photos put that joint at
`ANGLE_SET = 1000`.

The RH56DFTP user manual, section 2.6.11 (printed pages 21-22), defines one chord
angle per DOF measured against the metacarpal plane: α (fingers), θ (thumb
bending) and β (thumb rotation). Open, it gives α = 176°, θ = 70° and β = 165°;
this model gives 172.3°, 71.4° and 162.7° at `q = 0`. Closed, the real hand stops
well short of the manual's angles (table below).

> **The zero moved.** Earlier versions put `q = 0` on the manual's open angles,
> where the photographed open hand sat at `*_1` 0.0571, `*_2` 0.0108, `thumb_1`
> 0.1257, `thumb_2` −0.2074, `thumb_3` −0.0868 and `thumb_swing` 0.0400 rad. The
> joint origins were rotated by exactly those angles about each joint's axis. No
> pose changed, but joint values saved with the earlier model convert as
> `q_now = q_before − offset`.

With `q = 0` on the photographed open hand, the `q_max` and the multipliers put
each digit exactly where the photos put it, both fully open and fully closed:

| DOF | `q_max` (rad) | Mimic multiplier | Closed angle | Manual closed angle |
|---|---|---|---|---|
| index, middle, ring, little | **1.517** | `*_2`: **0.890** | α 34°–40° | α = 20° |
| thumb bending | **0.624** | `thumb_2`: **0.601**, `thumb_3`: **0.579** | θ 19.8° | θ = −13° |
| thumb rotation | **1.200** | — | β 94.0° | β = 90° |

Mimic joints carry `multiplier × q_max`. The four fingers share one curve (their
measurements agree within the noise; closed, they range from 1.42 to 1.57).

A mimic is linear and the real coupling is not: the finger's distal phalanx lags
at mid travel (q2/q1 ≈ 0.67 at `ANGLE_SET = 500`, 0.89 closed). So the mapping
below does not give the real proximal angle; it gives, for each command, the
actuated value that best places the fingertip with these multipliers (at
`ANGLE_SET = 500` the real finger is at 0.975 / 0.658, the table gives 0.889 with
0.791 distal). At its points the fingertips land within 2.0 mm (fingers) and
0.2 mm (thumb) of the real ones, exactly on them fully open and fully closed;
between points, within 3.1 mm (fingers) and 2.9 mm (thumb, rotation included).
The vendor's finger multiplier 1.05 folded the distal phalanx into the palm; the
earlier values with `q = 0` on the manual's angles (1.574 / 0.865 and
0.75 / 0.224 / 0.367; before them 1.62 / 0.85 and 0.66 / 0.40 / 0.60, fitted to
three photos per DOF) left the fingertips up to 7 and 14 mm off. For the real
joint angles, use **Software coupling**.

**Mapping the SDK scale.** `ANGLE_SET` is *not* linear in the joints: the real
finger's proximal joint covers 64 % of its travel by `ANGLE_SET = 500` (the distal
49 %), the thumb bending and rotation 57 %. For `coupling:=mimic`,
interpolate the actuated joints linearly between these points, with
`s = 1 − ANGLE_SET/1000` (thumb rotation within 3.5° of the real hand):

| `s` | 0 (open) | 0.25 | 0.5 | 0.75 | 1 (closed) |
|---|---|---|---|---|---|
| `*_index/middle/ring/little_1` | 0 | 0.519 | 0.889 | 1.194 | 1.517 |
| `*_thumb_1` | 0 | 0.159 | 0.334 | 0.497 | 0.624 |
| `*_thumb_swing` | 0 | 0.320 | 0.680 | 1.000 | 1.200 |

```python
import numpy as np

S = [0.0, 0.25, 0.5, 0.75, 1.0]
Q = {'finger':      [0.0, 0.519, 0.889, 1.194, 1.517],
     'thumb_1':     [0.0, 0.159, 0.334, 0.497, 0.624],
     'thumb_swing': [0.0, 0.320, 0.680, 1.000, 1.200]}

def angle_set_to_q(angle_set, dof):        # hand -> URDF (mimic variant)
    return float(np.interp(1 - angle_set / 1000.0, S, Q[dof]))

def q_to_angle_set(q, dof):                # URDF -> hand
    return int(round(1000 * (1 - np.interp(q, Q[dof], S))))
```

A complete implementation for all six DOF, with the inverse, URDF joint names as
input and a self-test that checks the table against these limits, lives in
`h1_2_joint_control/scripts/selector_poses_manos/conversion_angle_set.py` (still
with the earlier three-point table and zero; it needs these values).

The **right hand's digits are the exact mirror of the left hand's** across the
palm's `y=0` plane — verified to 0.000000 mm vertex-for-vertex, at rest and
throughout the joint range. The left hand is the reference: to change the digit
geometry, edit `inspire_hand_left.urdf.xacro` and re-mirror the right, never the
other way round. The palm (`*_hand_base_link`) keeps each side's own vendor mesh.

Both palms declare the **same inertial, set from a scale reading of the real
hand**: **0.840 kg** for the whole hand, wrist flange included. The 12 digits keep
their exported 0.143381 kg, so the palm carries **0.696619 kg**. The flange in
the palm mesh is the short one the robot carries (see *Hand mounting points*).

The deficit was put entirely in the palm rather than spread over the hand. The
CAD export left almost every link at SolidWorks' default 1000 kg/m³ — palm,
`*_thumb_swing` and all eight finger phalanges — so the exported 457 g was never
a measurement. The palm is where the six actuators, gearboxes, controller and
wiring sit, which is exactly what such an export omits, and it lands at
2682 kg/m³, an aluminium housing packed with motors. Scaling the whole hand by
1.8362 instead would drive `*_thumb_1` and `*_thumb_2` to 9 900 and 10 400 kg/m³,
denser than steel, so that option is not physical. The centre of mass is
unchanged and the tensor is scaled by 0.696619/0.314091 = 2.217891, preserving
the vendor's mass distribution.

| Side | Actuated | Mimic (ROS) / Independent (Isaac) |
|---|---|---|
| Left | `left_thumb_swing`, `left_thumb_1`, `left_index_1`, `left_middle_1`, `left_ring_1`, `left_little_1` | `*_2/_3` joints |
| Right | `right_thumb_swing`, `right_thumb_1`, `right_index_1`, `right_middle_1`, `right_ring_1`, `right_little_1` | `*_2/_3` joints |

### Hand mounting points

```
left_wrist_yaw_link  ──(fixed, xyz=0.161 -0.1125 -0.0075, rpy=π π/2 π)──► left_hand_base_link
right_wrist_yaw_link ──(fixed, xyz=0.161  0.1125 -0.0075, rpy=0  π/2 0)──► right_hand_base_link
```

From the wrist outwards, the real joint is made of three parts, and the model
now has all three:

| Part | Size | In the model |
|---|---|---|
| End of `*_wrist_yaw_link` | Ø56 face at `x = 0.054` | Unitree's mesh, unchanged |
| Adapter ring, bolted on that face | Ø56 × 10 mm, `x = 0.054 … 0.064` | **added** to `meshes/h1_2/wrist_yaw_link.STL` |
| Hand flange (RH56DFTP-2), on the ring | Ø50 × 16 mm, `x = 0.064 … 0.080` | **shortened** in both `base_link.STL` |

The flange is a cup held by four radial screws; its end face sits on the outer
face of the ring, `x = 0.064`. The palm origin lies 80.96 mm above the top of the
flange, hence `x = 0.064 + 0.016 + 0.081 = 0.161`.

Two meshes differ from their vendors' because of this:

- **`base_link.STL`, both hands.** The vendor's palm is the RH56DFTP-0, with the
  standard Ø50 × 30 mm flange and its aviation connector. A 14 mm slice was cut
  out of the flange (the slice holding the connector recess) and the rim with
  its four radial screw holes was moved up against the rest. The connector went
  back in smaller and elsewhere: the vendor's Ø16 recess does not fit in 16 mm,
  so a Ø14 pocket, 3 mm deep like the vendor's, is cut in the wall halfway
  between the dorsal screw hole and the one on the little-finger side, where the
  real hand has it, with the vendor's connector piece in it scaled to Ø12.25.
  The flange is still a closed solid; every other triangle is byte for byte the
  vendor's.
- **`wrist_yaw_link.STL`.** Unitree's mesh has no adapter ring, so the hand
  appeared to sit 10 mm closer to the wrist than it does. The ring is appended as
  one more closed solid. Unitree's own `h1_2.urdf` has a 10 mm ring in the same
  place, as the first 10 mm of its Inspire hand mesh.

**Where the numbers come from.** A photo of the left wrist with the hand mounted
was compared with this model rendered in MuJoCo through a virtual camera fitted
to the photo: same focal length, its distance from the apparent diameters of the
ring and the flange, and its height from the wrist link alone (end face, rounded
end and yaw axis). The ring length and the flange height were then set so that
the seat, the edge between the flange and the hand's silver ring, and the screw
of that ring fall on the photographed ones. The screw lands within about 1 mm,
and a flange 1 mm taller or shorter shows in the overlay. With the vendor's
flange and `x = 0.165` the hand sat 4 mm too far out.

The three lengths were then measured on the real robot and agree: the ring is
10 mm, the flange 16 mm, and flange plus silver ring about 25 mm (24.6 in the
model).

The flange axis is 0.7 mm (y) and 0.8 mm (z) off the wrist axis, as before.

#### Regenerating the edited meshes

Both edits are scripted and take the vendors' original meshes, which are in
commit `45099e5`:

```bash
P=ros_h1_2_ws/src/h1_2_inspire_description
git show 45099e5:$P/meshes/hand_left/base_link.STL   > /tmp/left.STL
git show 45099e5:$P/meshes/hand_right/base_link.STL  > /tmp/right.STL
git show 45099e5:$P/meshes/h1_2/wrist_yaw_link.STL   > /tmp/wrist.STL
$P/tools/shorten_hand_flange.py   /tmp/left.STL  $P/meshes/hand_left/base_link.STL  16   # flange height, mm
$P/tools/shorten_hand_flange.py   /tmp/right.STL $P/meshes/hand_right/base_link.STL 16
$P/tools/add_wrist_adapter_ring.py /tmp/wrist.STL $P/meshes/h1_2/wrist_yaw_link.STL 10   # ring length, mm
```

With another flange height `h` or ring length `r` (mm), the mount offset is
`x = 0.054 + (r + h + 80.96)/1000`, to be set in `h1_2_with_inspire_hands.urdf.xacro`
and in the two flat URDFs. The connector needs `h ≥ 16`; a shorter flange takes
`--no-connector`. After changing a palm mesh, split the colours again (below).

### Hand colours

The hands are drawn as the real ones: **black** flange, palm and rubber pads and
tips of the digits; **white** cover on the back of the hand and shells of the
digits; **silver** ring between palm and flange, connector, and the metal
brackets, rods and hinge lugs of the digits.

An STL carries no colour and a URDF visual takes one material, so each hand link
has one `<visual>` per colour, on a mesh of its own in `meshes/hand_*/visual/`
(`left_index_1_black.STL`, `left_index_1_white.STL`, …), with the materials
`inspire_black`, `inspire_white` and `inspire_silver`. The pieces add up to the
link mesh, and the link mesh itself stays as the collision geometry, so nothing
changes for collisions or dynamics. The file names carry the side because MuJoCo
names a URDF's meshes after their files.

`tools/split_hand_colors.py` writes the pieces from the link meshes:

```bash
ros_h1_2_ws/src/h1_2_inspire_description/tools/split_hand_colors.py
```

It cuts every mesh into smooth patches along its creases, which gives back the
faces of the CAD model — the dorsal cover, the rubber of the palm, the silver
ring, the pad and the shell of each finger are one patch each — colours a few of
them by rule and lets the rest take the colour of the neighbour they share the
longest border with. Two borders do not follow a crease, because the meshes have
none there:

- **The thumb.** Rubber and shell are one smooth surface, so the rubber ends on
  a plane, at a height read off photos of the real hand. The real edge is a
  curve.
- **The cover of the right palm.** The right palm is the vendor's own mesh, not
  the left one mirrored, and in it the cover runs into the body. Its outline is
  taken from the left palm.

Any consumer that picks the hand's visual geometry by counting geoms or meshes
per link now finds two or three where there was one.

### Mass

| Part | Mass (kg) | Source |
|---|---|---|
| Body, no hands | 66.9840 | upstream URDF, matches the real robot |
| Left hand (palm 0.696619 + digits 0.143381) | 0.8400 | weighed on a scale |
| Right hand (identical, mirrored) | 0.8400 | weighed on a scale |
| **Full robot with both hands** | **68.6640** | |

The body figure is the real robot's mass. Every link's mass, centre of mass and
inertia tensor in `h1_2_body.urdf.xacro` is identical to upstream
[`h1_2_handless.urdf`](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/h1_2_description)
— verified link by link, with the body meshes in use matching by checksum, all
but `wrist_yaw_link.STL`, which carries the hand's adapter ring (see *Hand
mounting points*). Only the mesh paths, the xacro namespace and the material
names differ, plus the `<mujoco>` compiler block, which this package omits.

> **Reading the mass back with Pinocchio.** On a fixed-base model,
> `pin.computeTotalMass()` returns **62.6810 kg**, not 68.6640 kg. It is not a
> defect in the model: `buildModelFromUrdf` without a root joint pins `pelvis` to
> the `universe` body (index 0), and the sum runs over bodies `1..njoints`, so the
> pelvis (5.983 kg) is left out. Add a free-flyer to get the real figure:
>
> ```python
> import pinocchio as pin
> model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
> pin.computeTotalMass(model)   # 68.664000
> ```

---

## Software coupling

A URDF `<mimic>` can only make a joint a fixed multiple of another, and the real
hand does not move that way. Measured on photos of the real hand at nine
`ANGLE_SET` values per DOF (0, 125, … 1000), the distal finger phalanx lags at
mid travel and catches up at the end, and the three thumb joints follow three
different curves. With the software coupling every hand joint follows its own
measured curve of the command. (The same photos set the joint zeros, which are
the open hand, and the mimic variant's `q_max` and multipliers, chosen so that
its open and closed poses are the real ones.)

| Variant | Hand joints in the URDF | Who computes the coupled joints | Error vs. the real hand |
|---|---|---|---|
| `coupling:=mimic` (xacro default) | 6 actuated + 6 `<mimic>` per hand | the URDF consumer, linearly | fingertips within 3 mm, joint angles up to 0.13 rad off |
| `coupling:=software` (launch default) | 12 independent per hand | `hand_coupling.py` / `hand_coupling_node.py` | the measurement's: rms 0.01–0.05 rad |

### The curves

`s = 1 − ANGLE_SET/1000` (0 open, 1 closed; `ANGLE_ACT` reads on the same scale).
The four fingers share one curve per phalanx: their measurements agree within
the noise (rms 0.05 rad around the common curve). Values in rad:

| `ANGLE_SET` | finger `*_1` | finger `*_2` | `thumb_1` | `thumb_2` | `thumb_3` | `thumb_swing` |
|---|---|---|---|---|---|---|
| 1000 (open) | 0 | 0 | 0 | 0 | 0 | 0 |
| 750 | 0.599 | 0.334 | 0.183 | 0.073 | 0.062 | 0.320 |
| 500 | 0.975 | 0.658 | 0.355 | 0.200 | 0.136 | 0.680 |
| 250 | 1.243 | 0.991 | 0.505 | 0.321 | 0.232 | 1.000 |
| 0 (closed) | 1.517 | 1.350 | 0.624 | 0.375 | 0.361 | 1.200 |

The full curves (21 nodes, linear in between) are `config/hand_coupling.yaml`.
They are fitted by `tools/fit_hand_coupling.py` to
`config/hand_coupling_measurements.yaml`: weighted cubics constrained to grow
for the fingers and the thumb bending, monotone interpolation through the points
for the thumb rotation (precise to 0.01 rad, with a sharp knee near closure).
Every curve starts at 0, the open hand: the cubics are fitted through the origin.
Measurements, method and uncertainties are documented in that file. To refit
after new photos:

```bash
python3 tools/fit_hand_coupling.py --plot /tmp/curves.png   # rewrites config/hand_coupling.yaml
python3 tools/fit_hand_coupling.py --check                  # is the file up to date?
```

The tool also reports where the measurements alone put the open pose, and warns
if it drifts more than 0.02 rad from `q = 0`. Then the zero no longer matches the
hand: rotate the joint origins by the drift and subtract it from the
measurements, as was done when the zero moved (see the hands section above).
With `coupling:=software` every joint's limits are its curve's range, from 0 to
the closed value (`limits:` in the YAML, read by the xacro).

### In Python

```python
import sys; sys.path.insert(0, "<package>/scripts")
from hand_coupling import HandCoupling

hc = HandCoupling()                     # reads config/hand_coupling.yaml
# SDK order: little, ring, middle, index, thumb_bend, thumb_rot
q = hc.joints_from_angle_set([1000, 1000, 500, 500, 1000, 0], "left")
# {'left_little_1_joint': 0.0, ..., 'left_thumb_swing_joint': 1.2}  (12 joints)
hc.angle_set_from_joints(q, "left")     # [1000, 1000, 500, 500, 1000, 0], from the actuated joints
hc.couple({...6 actuated joints...}, "left")   # the 12 joints from the 6 actuated ones
hc.derivatives(s, "left")               # {joint: (DOF index, dq/ds)}, for Jacobians in s
```

From a terminal: `python3 scripts/hand_coupling.py 1000 1000 500 500 1000 0 --side left`,
`--table` for the curves every 125, `--test` for the self-test.

### In ROS 2

`hand_coupling_node.py` publishes the 12 joints of each hand on `joint_states`
from either input:

| Input | Type | Use |
|---|---|---|
| `joint_states_input` | `sensor_msgs/JointState` | a robot state whose hands carry only the 6 actuated joints; every other joint passes through, the 6 coupled ones of each hand are computed |
| `hand_angle_set/left`, `hand_angle_set/right` | `std_msgs/Int32MultiArray` | the 6 `ANGLE_SET` or `ANGLE_ACT` values of one hand, SDK order; to mirror the real hands |

Parameters: `input_topic`, `output_topic`, `angle_set_topic` (prefix), `config`.

```bash
ros2 run h1_2_inspire_description hand_coupling_node.py
ros2 topic pub --once /hand_angle_set/left std_msgs/msg/Int32MultiArray "{data: [1000, 1000, 500, 500, 1000, 0]}"
```

The three display launches with hands take `coupling:=software` (default) or
`coupling:=mimic`. With `software`, robot_state_publisher gets the URDF without
`<mimic>`; joint_state_publisher_gui shows 6 sliders per hand (the coupled joints
are declared as its `dependent_joints`, so they get no slider) and publishes on
`joint_states_input`; the node fills in the hands and publishes `joint_states`.
Each slider is the actuated joint's angle, over its curve's range (the software
variant's limits); a value outside it is clamped to the curve's end.

### Pinocchio and MuJoCo

`urdf/h1_2_with_RH56DFTP_hands_software_coupling.urdf` is the flat URDF of the
software variant: 51 independent revolute joints (`nq = 58`, `nv = 57` with the
free flyer, same 68.664 kg). Set the hands with `joints_from_angle_set()`; for a
Jacobian in the 6 commands of a hand, combine the joint columns:

```python
import numpy as np, pinocchio as pin
model = pin.buildModelFromUrdf(path_software, pin.JointModelFreeFlyer())
data = model.createData()
q = pin.neutral(model)
for name, value in hc.joints_from_s(s, "left").items():
    q[model.joints[model.getJointId(name)].idx_q] = value
pin.computeJointJacobians(model, data, q)
pin.framesForwardKinematics(model, data, q)
J = pin.getFrameJacobian(model, data, model.getFrameId("left_index_2"), pin.LOCAL_WORLD_ALIGNED)
J_s = np.zeros((6, 6))                  # fingertip twist vs. the 6 commands s of the left hand
for name, (k, dq_ds) in hc.derivatives(s, "left").items():
    J_s[:, k] += J[:, model.joints[model.getJointId(name)].idx_v] * dq_ds
```

Its geometry is the mimic model's: with the coupled joints set to the mimic
multiples, every frame lands in the same place (0 m difference).

MuJoCo can hold the nonlinear coupling itself: `<equality joint polycoef>`
couples two joints by a quartic. `h1_2_mujoco/modelo_h1_2_manos.py` (outside
this package) adds one per coupled joint when it loads this URDF, with the
quartic from `HandCoupling.polynomial()`, within 0.007 rad of the curves.

### Regenerating after a change

```bash
python3 tools/fit_hand_coupling.py                    # if the measurements changed
xacro urdf/h1_2_with_inspire_hands.urdf.xacro coupling:=software \
      -o urdf/h1_2_with_RH56DFTP_hands_software_coupling.urdf
```

then restore the flat file's header comment and robot name
(`h1_2_with_RH56DFTP_hands_software_coupling`), as for the mimic one.

## ROS 2 Installation and Usage

```bash
cd ~/ros2_ws/src
git clone https://github.com/smorales2405/h1_2_inspire_description.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select h1_2_inspire_description
source install/setup.bash
```

### Visualization in RViz

```bash
# Full robot with both hands (joint sliders); hands with the software coupling
ros2 launch h1_2_inspire_description display_h1_2_with_hands.launch.py
# ... or with the <mimic> tags
ros2 launch h1_2_inspire_description display_h1_2_with_hands.launch.py coupling:=mimic

# H1-2 body only
ros2 launch h1_2_inspire_description display_h1_2.launch.py

# Left hand only (also takes coupling:=mimic)
ros2 launch h1_2_inspire_description display_hand_left.launch.py

# Right hand only
ros2 launch h1_2_inspire_description display_hand_right.launch.py
```

### Xacro architecture

```
h1_2_with_inspire_hands.urdf.xacro
├── xacro:include → h1_2_body.urdf.xacro
├── xacro:include → inspire_hand_left.urdf.xacro
├── xacro:include → inspire_hand_right.urdf.xacro
├── joint: left_inspire_hand_mount_joint  (fixed)
└── joint: right_inspire_hand_mount_joint (fixed)
```

---

## Using the model with Pinocchio

For the software coupling use `urdf/h1_2_with_RH56DFTP_hands_software_coupling.urdf`
instead, without `mimic=True`; see **Software coupling › Pinocchio and MuJoCo**.
What follows is the mimic variant.

`urdf/h1_2_with_RH56DFTP_hands.urdf` is the xacro tree flattened into a single
plain URDF, for Pinocchio and anything else that reads URDF but not xacro. It is
the same model bit for bit: joint and frame names, inertial parameters, joint
placements and limits all compare equal, and forward kinematics over random
configurations agrees to 0.

```python
import pinocchio as pin

path = "/path/to/your_ws/src/h1_2_inspire_description/urdf/h1_2_with_RH56DFTP_hands.urdf"
model = pin.buildModelFromUrdf(path, pin.JointModelFreeFlyer(), mimic=True)
data  = model.createData()
# model.nq == 46, model.nv == 45, pin.computeTotalMass(model) == 68.664

visual = pin.buildGeomFromUrdf(model, path, pin.GeometryType.VISUAL,
                               package_dirs=["/path/to/your_ws/src"])
collision = pin.buildGeomFromUrdf(model, path, pin.GeometryType.COLLISION,
                                  package_dirs=["/path/to/your_ws/src"])
# 93 visual objects (the hand links have one per colour), 48 collision objects
```

`urdf/h1_2_handless.urdf` is the same file with both hands removed — the 26
links and 26 joints hanging off the two mount joints, those joints included —
for when the hands are not part of the problem:

```python
model = pin.buildModelFromUrdf(handless_path, pin.JointModelFreeFlyer())
# model.nq == 34, model.nv == 33, pin.computeTotalMass(model) == 66.984
```

It has no mimic joints, so the `mimic` argument makes no difference there. Its
model compares equal to expanding `h1_2_body.urdf.xacro`, and equal to upstream
`h1_2_handless.urdf`, on joint names, inertial parameters, joint placements,
limits and forward kinematics over random configurations.

Two arguments are easy to leave out and both change the answer:

| Argument | If omitted |
|---|---|
| `mimic=True` | The 12 coupled phalanges become independent joints and `nv` goes from 39 to 51. Commanding a finger no longer moves its distal phalanx. |
| `pin.JointModelFreeFlyer()` | `pelvis` is welded to the `universe` body, so `computeTotalMass` skips its 5.983 kg and reports 62.681 kg. |

With both, the articulated count is `nv - 6 = 39`: 27 body joints plus 6 actuated
joints per hand.

Three links carry no geometry at all (`imu_link`, `camera_link`, `lidar_link`),
and seven more carry no collision mesh (`*_hip_yaw_link`, `*_ankle_pitch_link`,
`*_wrist_yaw_link`, `logo_link`). That is how the upstream Unitree URDF ships and
is left as is.

### Regenerating it

The file is generated, not maintained by hand. After any change to the xacro
sources:

```bash
xacro urdf/h1_2_with_inspire_hands.urdf.xacro -o urdf/h1_2_with_RH56DFTP_hands.urdf
xacro urdf/h1_2_body.urdf.xacro               -o urdf/h1_2_handless.urdf
```

then restore each header comment and set the robot name
(`h1_2_with_RH56DFTP_hands`, `h1_2_handless`), neither of which xacro writes.

---

## Isaac Sim 5.1 Import Guide

Use the file `urdf/h1_2_hands_isaac.urdf` — a flat URDF pre-processed for Isaac Sim 5.1.

### What was changed from the Xacro-based URDF

| # | Change | Detail |
|---|---|---|
| 1 | Inertial added to 4 sensor/logo links | `logo_link`, `imu_link`, `camera_link`, `lidar_link` — mass=0.001 kg, I=1e-6 kg·m² |
| 2 | Collision added to 6 structural body links | `left/right_hip_yaw_link`, `left/right_ankle_pitch_link`, `left/right_wrist_yaw_link` — visual STL reused |
| 3 | 12 `<mimic>` tags removed | All hand secondary phalanges become independent revolute joints at q=0 |

No material name is empty any more: the hand visuals use `inspire_black`,
`inspire_white` and `inspire_silver` in the xacro sources too.

### Prerequisites

- Isaac Sim 5.1 installed
- Extension **`isaacsim.asset.importer.urdf`** enabled — verify via **Window › Extensions**, search "urdf"
- The ROS 2 workspace must be built (`colcon build`) before launching Isaac Sim, **or** the package path must be set manually in the importer (see step 5 below)

### Step-by-step import

1. **Launch Isaac Sim 5.1**

2. **Open the URDF Importer panel**  
   Go to **File › Import** (or **Isaac Utils › Workflows › URDF Importer** if the panel is docked)

3. **Select the file**  
   Navigate to and select:
   ```
   <your_workspace>/src/h1_2_inspire_description/urdf/h1_2_hands_isaac.urdf
   ```

4. **Configure Import Options**

   | Option | Recommended | Notes |
   |---|---|---|
   | **Fix Base** | OFF (Moveable base) | Use ON only for static tests without locomotion |
   | **Self Collision** | **OFF** | Enable only after verifying collision geometry |
   | **Create Physics Scene** | ON | Required for simulation |
   | **Distance Scale** | 1.0 | URDF is already in meters |
   | **Joint Drive Type** | None or Position | Use None for passive display |
   | **Default Drive Stiffness** | 100–1000 | Tune per-joint after import |

5. **Configure ROS Package Path** (if meshes fail to resolve)

   In the importer, find the **ROS Package List** field and add:

   | Package name | Path |
   |---|---|
   | `h1_2_inspire_description` | `/home/<user>/ros2_ws/src/h1_2_inspire_description` |

   Alternatively, source your workspace before launching Isaac Sim:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ./isaac-sim.sh
   ```

6. **Click Import**

7. **Check the Output Log** (`Window › Output Log`) for any mesh resolution errors or physics warnings

8. **Save as USD**  
   Once the robot loads correctly:  
   **File › Save As** → save as `.usd` to avoid re-importing from URDF in future sessions

### Recommended initial checks after import

- [ ] **Robot Model**: verify pelvis is the root, arms and legs attach correctly
- [ ] **Finger joints**: check that all 24 hand joints (12 per hand) are present in the Articulation Inspector
- [ ] **Mimic joints**: the 12 secondary phalanges (`*_2`, `*_3`) are now **independent revolute joints** at position 0 (fully open). They will not follow the primary joints automatically — see note below
- [ ] **Self-collision OFF**: do not enable until collision meshes have been manually inspected in the viewport
- [ ] **Articulation root**: confirm that `pelvis` is set as the articulation root (`RigidBodyAPI` + `ArticulationRootAPI`)
- [ ] **Convert to USD**: after a successful import, save as USD for faster future loading

### Reconstructing mimic (finger coupling) in Isaac Sim

Isaac Sim does not support the URDF `<mimic>` tag natively. After importing, you have two options:

**Option A — Simple control (recommended for initial tests)**  
Drive only the 6 primary joints per hand. Leave secondary phalanges at q=0 (open). This is the default after import.

**Option B — Full coupling via Python**  
After importing, attach an `ArticulationController` script to the robot prim:

```python
from omni.isaac.core.articulations import Articulation

robot = Articulation("/World/h1_2_with_inspire_hands")
robot.initialize()

# Example: couple left_index_2 to left_index_1 (multiplier 0.890)
def apply_mimic(primary_joint: str, mimic_joint: str, multiplier: float):
    idx_p = robot.get_dof_index(primary_joint)
    idx_m = robot.get_dof_index(mimic_joint)
    q = robot.get_joint_positions()
    q[idx_m] = multiplier * q[idx_p]
    robot.set_joint_positions(q)

# Call apply_mimic() each simulation step for all 12 coupled pairs
```

For the measured, nonlinear coupling instead of the multipliers below, set the
12 joints of each hand from `HandCoupling.couple()` (from the 6 actuated joints)
or `HandCoupling.joints_from_angle_set()` (from the hand's commands); see
**Software coupling**. `h1_2_hands_isaac.urdf` has the mimic variant's limits;
the curves start at the same 0 and end within 0.0003 rad of them.

Complete coupling table:

| Mimic joint | Drives from | Multiplier |
|---|---|---|
| `left_thumb_2_joint` | `left_thumb_1_joint` | 0.601 |
| `left_thumb_3_joint` | `left_thumb_1_joint` | 0.579 |
| `left_index_2_joint` | `left_index_1_joint` | 0.890 |
| `left_middle_2_joint` | `left_middle_1_joint` | 0.890 |
| `left_ring_2_joint` | `left_ring_1_joint` | 0.890 |
| `left_little_2_joint` | `left_little_1_joint` | 0.890 |
| `right_thumb_2_joint` | `right_thumb_1_joint` | 0.601 |
| `right_thumb_3_joint` | `right_thumb_1_joint` | 0.579 |
| `right_index_2_joint` | `right_index_1_joint` | 0.890 |
| `right_middle_2_joint` | `right_middle_1_joint` | 0.890 |
| `right_ring_2_joint` | `right_ring_1_joint` | 0.890 |
| `right_little_2_joint` | `right_little_1_joint` | 0.890 |

### Common problems and solutions

| Problem | Cause | Solution |
|---|---|---|
| Meshes not found / pink robot | `package://` path unresolved | Add `h1_2_inspire_description` path to the **ROS Package List** in the importer, or source the workspace before launching Isaac Sim |
| `pelvis` floats away on start | Floating base with gravity, no contacts | Set **Fix Base ON** for static tests, or add a ground plane and a PD controller |
| Physics explodes / instability | Large inertia ratio between body and finger links | Disable physics on sensor links (`imu_link`, `camera_link`, `lidar_link`) via the USD property panel |
| Finger secondary phalanges don't move | Mimic joints were removed | Expected — use Option A or B above to re-couple them |
| `logo_link` / sensor links have no collision | Intentional | These are visual/sensor-only frames; do not add collision |
| Wrist/hip collision unstable | STL mesh used for collision (complex geometry) | Replace collision on `left/right_wrist_yaw_link` with a `<cylinder>` primitive (~r=0.03, l=0.03) |
| Import warning: "no inertia" | Isaac version sensitivity | Already fixed in `h1_2_hands_isaac.urdf`; if it persists, increase mass of sensor links to 0.01 kg |
| USD shows T-pose only | No joint drives or states | Normal for a static load — use the **Articulation Inspector** (`Window › Physics › Articulation Inspector`) to manually move joints |

---

## Sources

- H1-2 URDF: adapted from [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) via [oscar-ramos/h1_2_utec](https://github.com/oscar-ramos/h1_2_utec)
- Inspire RH56DFTP URDF: exported from SolidWorks via [sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter)

## Maintainer

Sergio Morales — smorales@utec.edu.pe  
Universidad de Ingeniería y Tecnología (UTEC), Lima, Peru
