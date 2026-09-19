"""Tabla de articulaciones del Unitree H1-2 (27 motores comandables).

Fuentes de cada dato, para que se pueda auditar:

* **Índices**: `unitree_sdk2/example/h1/high_level/h1_2_arm_sdk_dds_example.cpp`
  y `unitree_ros2/example/src/src/h1-2/lowlevel/low_level_ctrl_hg.cpp`.
  Ambos coinciden.
* **Límites, par y velocidad**: `h1_2_handless.urdf` de `h1_2_utec/h1_2_description`
  (el mismo URDF que usa el resto del laboratorio). Los valores van SIN redondear:
  redondear 0.523598 a 0.524 dejaba el tope por FUERA del que declara el URDF.
  La tabla se verificó contra `h1_2_handless.urdf`: los topes son los del
  URDF sin redondear, porque redondear tres de ellos los dejó FUERA.
* **Grupos**: derivados del par máximo, que en el H1-2 identifica la familia de
  actuador.

⚠ Discrepancia conocida en las PIERNAS: `xr_teleoperate`
(`teleop/robot_control/robot_arm.py`, `H1_2_JointIndex`) llama al motor 1
`kLeftHipRoll` y al 2 `kLeftHipPitch`, al revés que el ejemplo oficial de
Unitree y que el URDF. Solo afecta a las piernas; los 14 motores de los brazos
coinciden en las tres fuentes. Aquí se usa el orden oficial (URDF).
"""
from __future__ import annotations

from dataclasses import dataclass

# Un LowCmd de la serie `hg` lleva 35 MotorCmd; el H1-2 solo usa 0..26.
NUM_CMD_MOTOR = 27
NUM_STATE_MOTOR = 35

# Índice del "motor" que NO existe y que el servicio `arm_sdk` reinterpreta
# como el peso de mezcla (0 = manda el robot, 1 = mandamos nosotros).
WEIGHT_INDEX = 27


@dataclass(frozen=True)
class Joint:
    idx: int
    name: str          # nombre corto, el que se pasa por línea de comandos
    urdf: str          # nombre en el URDF
    group: str         # leg | waist | shoulder | elbow | wrist
    q_min: float       # rad
    q_max: float       # rad
    tau_max: float     # Nm, límite del URDF
    dq_max: float      # rad/s, límite del URDF

    @property
    def side(self) -> str:
        if self.name.startswith("L_"):
            return "left"
        if self.name.startswith("R_"):
            return "right"
        return "center"

    def clamp(self, q: float, margin: float = 0.0) -> float:
        """Recorta q a los límites del URDF, dejando `margin` rad de guarda."""
        return min(max(q, self.q_min + margin), self.q_max - margin)


JOINTS: tuple[Joint, ...] = (
    # --- pierna izquierda -------------------------------------------------
    Joint(0,  "L_hip_yaw",        "left_hip_yaw_joint",         "leg",   -0.430,  0.430, 200.0, 23.0),
    Joint(1,  "L_hip_pitch",      "left_hip_pitch_joint",       "leg",   -3.140,  2.500, 200.0, 23.0),
    Joint(2,  "L_hip_roll",       "left_hip_roll_joint",        "leg",   -0.430,  3.140, 200.0, 23.0),
    Joint(3,  "L_knee",           "left_knee_joint",            "leg",   -0.120,  2.190, 300.0, 14.0),
    Joint(4,  "L_ankle_pitch",    "left_ankle_pitch_joint",     "leg",   -0.897334,  0.523598,  60.0,  9.0),
    Joint(5,  "L_ankle_roll",     "left_ankle_roll_joint",      "leg",   -0.261799,  0.261799,  40.0,  9.0),
    # --- pierna derecha ---------------------------------------------------
    Joint(6,  "R_hip_yaw",        "right_hip_yaw_joint",        "leg",   -0.430,  0.430, 200.0, 23.0),
    Joint(7,  "R_hip_pitch",      "right_hip_pitch_joint",      "leg",   -3.140,  2.500, 200.0, 23.0),
    Joint(8,  "R_hip_roll",       "right_hip_roll_joint",       "leg",   -3.140,  0.430, 200.0, 23.0),
    Joint(9,  "R_knee",           "right_knee_joint",           "leg",   -0.120,  2.190, 300.0, 14.0),
    Joint(10, "R_ankle_pitch",    "right_ankle_pitch_joint",    "leg",   -0.897334,  0.523598,  60.0,  9.0),
    Joint(11, "R_ankle_roll",     "right_ankle_roll_joint",     "leg",   -0.261799,  0.261799,  40.0,  9.0),
    # --- cintura ----------------------------------------------------------
    Joint(12, "waist_yaw",        "torso_joint",                "waist", -2.350,  2.350, 200.0, 23.0),
    # --- brazo izquierdo --------------------------------------------------
    Joint(13, "L_shoulder_pitch", "left_shoulder_pitch_joint",  "shoulder", -3.140,  1.570, 40.0,  9.0),
    Joint(14, "L_shoulder_roll",  "left_shoulder_roll_joint",   "shoulder", -0.380,  3.400, 40.0,  9.0),
    Joint(15, "L_shoulder_yaw",   "left_shoulder_yaw_joint",    "shoulder", -2.660,  3.010, 18.0, 20.0),
    Joint(16, "L_elbow",          "left_elbow_joint",           "elbow",    -0.950,  3.180, 18.0, 20.0),
    Joint(17, "L_wrist_roll",     "left_wrist_roll_joint",      "wrist",    -3.010,  2.750, 19.0, 31.4),
    Joint(18, "L_wrist_pitch",    "left_wrist_pitch_joint",     "wrist",    -0.4625,  0.4625, 19.0, 31.4),
    Joint(19, "L_wrist_yaw",      "left_wrist_yaw_joint",       "wrist",    -1.270,  1.270, 19.0, 31.4),
    # --- brazo derecho ----------------------------------------------------
    Joint(20, "R_shoulder_pitch", "right_shoulder_pitch_joint", "shoulder", -3.140,  1.570, 40.0,  9.0),
    Joint(21, "R_shoulder_roll",  "right_shoulder_roll_joint",  "shoulder", -3.400,  0.380, 40.0,  9.0),
    Joint(22, "R_shoulder_yaw",   "right_shoulder_yaw_joint",   "shoulder", -3.010,  2.660, 18.0, 20.0),
    Joint(23, "R_elbow",          "right_elbow_joint",          "elbow",    -0.950,  3.180, 18.0, 20.0),
    Joint(24, "R_wrist_roll",     "right_wrist_roll_joint",     "wrist",    -2.750,  3.010, 19.0, 31.4),
    Joint(25, "R_wrist_pitch",    "right_wrist_pitch_joint",    "wrist",    -0.4625,  0.4625, 19.0, 31.4),
    Joint(26, "R_wrist_yaw",      "right_wrist_yaw_joint",      "wrist",    -1.270,  1.270, 19.0, 31.4),
)

BY_NAME = {j.name: j for j in JOINTS}
BY_INDEX = {j.idx: j for j in JOINTS}

# Los 14 motores de los brazos, en el orden que usa `xr_teleoperate`.
ARM_INDICES = tuple(range(13, 27))
ARM_NAMES = tuple(BY_INDEX[i].name for i in ARM_INDICES)

# Lo que acepta el canal `arm_sdk`: los 14 del brazo MÁS la cintura.
# Fuente: `h1_2_arm_sdk_dds_example.cpp`, array `arm_joints` (15 elementos).
ARM_SDK_INDICES = ARM_INDICES + (12,)

LEG_INDICES = tuple(range(0, 12))


def resolve(spec: str) -> list[int]:
    """Traduce una especificación de línea de comandos a índices de motor.

    Acepta: un nombre (`L_elbow`), un índice (`16`), un grupo (`wrist`),
    un lado (`left`), o los alias `arms`, `left_arm`, `right_arm`, `arm_sdk`,
    `all`. También listas separadas por comas.
    """
    out: list[int] = []
    for token in (t.strip() for t in spec.split(",") if t.strip()):
        if token in BY_NAME:
            out.append(BY_NAME[token].idx)
        elif token.isdigit() and int(token) in BY_INDEX:
            out.append(int(token))
        elif token == "arms":
            out += list(ARM_INDICES)
        elif token == "arm_sdk":
            out += list(ARM_SDK_INDICES)
        elif token == "left_arm":
            out += [i for i in ARM_INDICES if BY_INDEX[i].side == "left"]
        elif token == "right_arm":
            out += [i for i in ARM_INDICES if BY_INDEX[i].side == "right"]
        elif token == "all":
            out += [j.idx for j in JOINTS]
        elif token in {j.group for j in JOINTS}:
            out += [j.idx for j in JOINTS if j.group == token]
        elif token in {"left", "right"}:
            out += [j.idx for j in JOINTS if j.side == token]
        else:
            raise ValueError(
                f"no reconozco '{token}'. Nombres válidos: "
                + ", ".join(j.name for j in JOINTS)
                + " | grupos: leg, waist, shoulder, elbow, wrist"
                + " | alias: arms, arm_sdk, left_arm, right_arm, left, right, all"
            )
    # sin duplicados, conservando el orden de aparición
    seen, uniq = set(), []
    for i in out:
        if i not in seen:
            seen.add(i)
            uniq.append(i)
    return uniq
