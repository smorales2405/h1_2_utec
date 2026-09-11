#!/usr/bin/env python3
"""F3 — ¿cambian las ganancias óptimas con la postura del brazo?

Toda la sintonización de este repositorio se hizo en una sola postura: codo a
85° y brazo colgando. La matriz de inercia que ve el hombro y el brazo de
palanca de la gravedad cambian mucho con la configuración, así que la pregunta
es si un solo conjunto de ganancias sirve para todas.

Rejilla de `postures_f3` en `config/gains.yaml`:

    P1  pitch   0°  roll ±18°  codo 85°   reposo, la ya medida (referencia)
    P2  pitch −40°  roll ±25°  codo 60°   trabajo, alcance frontal a mesa
    P3  pitch −70°  roll ±35°  codo 20°   extendida, peor caso

Las tres están verificadas libres de colisión y dentro de los topes.

DECISIÓN, ESCRITA ANTES DE MIRAR LOS DATOS
──────────────────────────────────────────
    kp*_max / kp*_min  <  1.5  en todas   ->  un solo conjunto, elegido en P3
                                              (el peor caso), documentando lo
                                              que se degrada en P1 y P2
    razón >= 1.5 en alguna                ->  gain scheduling por postura,
                                              tabla de 3 puntos interpolada

Está aquí arriba a propósito: el criterio se fija antes de ver los números, no
después. El script la aplica solo y dice cuál sale.

    python3 scripts/17_f3_postura.py                    la rejilla entera
    python3 scripts/17_f3_postura.py --postures P1,P3   solo dos
    python3 scripts/17_f3_postura.py --joints L_elbow   una articulación
    python3 scripts/17_f3_postura.py --dry-run

Tiempo: 4 articulaciones × 3 posturas × 6 kp × 3 repeticiones ≈ 216 ensayos.
Contando las pausas y los cambios de postura, algo más de una hora de robot.
"""
from __future__ import annotations

import argparse
import importlib
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import (add_common_args, apply_test_posture, build_client,
                     confirm, joint_index)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX

_tune = importlib.import_module("03_tune")
_all = importlib.import_module("09_tune_all")

RAZON_UMBRAL = 1.5          # de la tabla de decisión, fijada de antemano
PICO_P1 = 7.8               # Nm de transitorio medidos en P1 (§13), amp 0.12
JOINTS_F3 = ("L_shoulder_pitch", "L_shoulder_roll", "L_elbow", "L_wrist_pitch")


# Múltiplos del kp de referencia de cada articulación. Cubren un factor 5.7,
# centrados en el valor que ya está en el conjunto sintonizado.
KP_MULT = (0.35, 0.5, 0.7, 1.0, 1.4, 2.0)

# Lo mismo para kd. Que llegue a 1.5× la referencia no es arbitrario: en el
# barrido que produjo `tuned_gff`, `L_shoulder_pitch` ganó con kd 20.25, que
# era el MÁXIMO explorado. Un ganador en el borde no es un óptimo, así que la
# rejilla tiene que pasar de ahí.
KD_MULT = (0.5, 1.0, 1.5)


def rejilla_de(idx, a, gains):
    """Candidatos (kp, kd) de una articulación, acotados por SATURACIÓN.

    El tope no es una precaución teórica. El criterio penaliza el error de
    seguimiento, y en una articulación con carga estática ese error es sobre
    todo caída por gravedad, `tau_g/kp`, que baja monótonamente con kp: un
    barrido sin tope elige siempre el kp más alto que se le ofrezca. Pasó con
    la lista del protocolo el 2026-09-10 —el codo ganó con kp 260 en las dos
    posturas y la razón salió 1.00, que no medía nada—.

    Lo que pone el límite es que un salto de consigna de `amp` radianes no
    llegue al umbral de par con el que aborta la seguridad:

        kp_max = tau_abort_fraction · tau_max / amp

    Para el codo con amp 0.12 son 105, y la lista del protocolo llegaba a 260.

    Si de la lista pedida sobreviven menos de cuatro, se construye una serie
    geométrica hasta el tope: mejor seis puntos dentro de la zona segura que
    dos, porque con dos no se puede ver dónde está el máximo.
    """
    j = BY_INDEX[idx]
    tope = _all.kp_maximo(j, gains.safety, a.amp)
    ref = gains.for_index(idx)[0]
    kps, fuera, nota = [], [], ""

    if a.kp_list:
        pedidos = _tune.floats(a.kp_list)
        kps = [k for k in pedidos if k <= tope]
        fuera = [k for k in pedidos if k > tope]
        nota = f"lista pedida, tope {tope:.0f}"
        if len(kps) < 4:
            kps, fuera = [], []
            nota = f"la lista pedida no deja 4 bajo el tope {tope:.0f}"

    if not kps:
        # Centrada en el kp que la articulación YA tiene, no en una lista
        # absoluta igual para todas. Una lista fija no sabe dónde está el
        # valor bueno de cada una: `shoulder_roll` vale 70 en `tuned_gff`, y
        # la lista del protocolo empezaba en 60, o sea un solo punto por
        # debajo. El barrido del 2026-09-10 ganó justo ahí, topando por abajo
        # sin poder seguir, y el resultado no valió.
        kps = sorted(round(m * ref, 1) for m in KP_MULT if m * ref <= tope)
        if any(m * ref > tope for m in KP_MULT):
            # El tope tiene que ESTAR en la rejilla. Si no, se pierde justo la
            # zona donde el criterio empuja: en el codo y la muñeca el ganador
            # fue el tope mismo (105 y 111).
            kps = sorted(set(kps + [round(tope, 1)]))
        if len(kps) < 4:                      # el tope corta muy pronto
            kps = sorted(round(tope * (0.25 ** (n / 5.0)), 1)
                         for n in range(5, -1, -1))
        nota = (nota + "; " if nota else "") + f"centrada en kp={ref:.0f}, tope {tope:.0f}"

    if fuera:
        print(f"  {j.name:<20}{nota}; fuera por saturación: "
              + ", ".join(f"{k:.0f}" for k in fuera))
    else:
        print(f"  {j.name:<20}{nota}")
    if a.kd_list:
        kds = _tune.floats(a.kd_list)
    else:
        kd_ref = gains.for_index(idx)[1]
        kds = sorted(round(m * kd_ref, 2) for m in KD_MULT)
        print(f"  {'':<20}kd centrado en {kd_ref:.2f}: "
              + ", ".join(f"{k:g}" for k in kds))
    return [(kp, kd) for kp in kps for kd in kds]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    # `legs="hold"` y no el `free` de por defecto. En F3 los brazos hacen
    # excursiones grandes —P3 los lleva a pitch −70°— y con el robot colgado
    # del arnés la reacción mueve el cuerpo. Observado el 2026-09-10: durante
    # P3 las caderas cambiaron de ángulo y las piernas se levantaron, con el
    # robot flotando en el arnés. En un ensayo de brazo solo deben moverse los
    # brazos, así que las piernas se sostienen.
    ap.set_defaults(channel="lowcmd", gains="tuned_gff", gravity_ff=True,
                    legs="hold")
    ap.add_argument("--postures", default=None,
                    help="lista separada por comas (por defecto, todas)")
    ap.add_argument("--joints", default=",".join(JOINTS_F3))
    ap.add_argument("--kp-list", default=None,
                    help="lista fija de kp. Por defecto se genera por "
                         "articulación alrededor de su kp de referencia, que "
                         "es lo que una lista absoluta no puede hacer bien.")
    ap.add_argument("--kd-list", default=None,
                    help="lista fija de kd. Por defecto se genera alrededor "
                         "del kd de referencia de cada articulación.")
    ap.add_argument("--traj", default="smooth_step",
                    help="smooth_step añade sobreimpulso y error final al "
                         "criterio, que es lo que pide F3")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--amp", type=float, default=0.12)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--cycles", type=float, default=3.0)
    ap.add_argument("--pause", type=float, default=0.6)
    ap.add_argument("--rise", type=float, default=0.3)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--f0", type=float, default=0.2)
    ap.add_argument("--f1", type=float, default=3.0)
    ap.add_argument("--w-err", type=float, default=1.0)
    ap.add_argument("--w-chatter", type=float, default=1.0)
    ap.add_argument("--w-overshoot", type=float, default=0.5)
    ap.add_argument("--zero-dq", action="store_true",
                    help="medir con dq_des = 0, como hacía xr_teleoperate sin "
                         "el parche. Por defecto se manda la velocidad de "
                         "referencia, que es el modo en que se sintonizó "
                         "`tuned_gff`.")
    a = ap.parse_args()

    gains = cfg.load(a.gains)
    posturas = ([p.strip() for p in a.postures.split(",")] if a.postures
                else gains.posture_names())
    for p in posturas:
        gains.posture(p)              # falla pronto si el nombre no existe
    articulaciones = [joint_index(n) for n in a.joints.split(",")]
    grids = {i: rejilla_de(i, a, gains) for i in articulaciones}

    n_ens = sum(len(grids[i]) for i in articulaciones) * len(posturas) * a.repeats
    print(f"\n  F3 · {len(posturas)} posturas × {len(articulaciones)} "
          f"articulaciones × {a.repeats} repeticiones  =  {n_ens} ensayos")
    print(f"  criterio de decisión, fijado de antemano: "
          f"razón kp* < {RAZON_UMBRAL} → un solo conjunto")

    # Aviso de par: en P3 la gravedad ya consume buena parte del margen.
    if gains.posture_names() and a.gravity_ff:
        try:
            from h1_2_joint_control.gravity import GravityModel
            import numpy as np
            gm = GravityModel(verbose=False)
            print(f"\n  {'postura':<8}{'tau gravedad en shoulder_pitch':>34}"
                  f"{'margen al aborto':>19}")
            for p in posturas:
                q = np.zeros(27)
                for i, v in gains.posture(p).items():
                    q[i] = v
                t = abs(gm.tau(q).get(13, 0.0))
                tope = gains.safety.tau_abort_fraction * BY_INDEX[13].tau_max
                # El transitorio del escalón se SUMA a esto. Medido en P1
                # con `tuned_gff`: 7.8 Nm de pico en shoulder_pitch con
                # amplitud 0.12 rad. Si el margen no da para eso, el aborto
                # por par saltará a mitad del barrido y se perderá el tiempo
                # de robot; es mejor bajar la amplitud que tocar el umbral.
                # Factor 1.3 sobre el transitorio medido, y no es prudencia
                # genérica: la identificación de gravedad se hizo alrededor de
                # P1, así que en P2 y P3 este número EXTRAPOLA. Si el modelo se
                # queda corto, lo que falta sale del margen.
                margen = tope - t
                aviso = ""
                if margen < PICO_P1 * 1.3:
                    amp_ok = a.amp * max(margen / (PICO_P1 * 1.3), 0.3)
                    aviso = (f"   ⚠ transitorio ~{PICO_P1:.1f} N: "
                             f"usa --amp {amp_ok:.2f} aquí")
                elif margen < PICO_P1 * 2.0:
                    aviso = "   margen justo, vigila el aborto"
                print(f"  {p:<8}{t:>32.1f} N{margen:>17.1f} N" + aviso)
        except Exception as exc:
            print(f"  (no se pudo estimar el par de gravedad: {exc})")

    confirm(a, "Los brazos recorren posturas muy distintas. Robot COLGADO.")

    stamp = rec.stamp()
    todos: dict[tuple[str, int], list] = {}
    for postura in posturas:
        for idx in articulaciones:
            j = BY_INDEX[idx]
            print(f"\n  ══ {postura} · {j.name} "
                  f"{'═' * max(0, 46 - len(postura) - len(j.name))}")
            a.posture = postura
            a.tag = postura        # entra en el nombre del CSV y en el índice
            g = cfg.load(a.gains)
            cli = build_client(a, [idx], verbose=False)
            resultados = []
            try:
                cli.wait_for_state()
                cli.engage()
                apply_test_posture(cli, a, g)
                _tune.barre_candidatos(cli, idx, a, g, grids[idx], stamp, resultados)
            except SafetyAbort as e:
                print(f"    ⚠ abortado por seguridad: {e}")
            except KeyboardInterrupt:
                print("\n  interrumpido por el usuario")
                cli.__exit__(None, None, None)
                return 1
            finally:
                cli.__exit__(None, None, None)
            deriva, quien = cli.leg_drift() if cli else (0.0, -1)
            if deriva > math.radians(2.0):
                print(f"    ⚠ las piernas se han movido: "
                      f"{BY_INDEX[quien].name} {math.degrees(deriva):.1f}°. "
                      f"En un ensayo de brazo no deberían.")
            if resultados:
                resultados.sort(key=lambda r: r[0])
                todos[(postura, idx)] = resultados
                J, kp, kd, t, st = resultados[0]
                print(f"    mejor: kp={kp:.1f} kd={kd:.2f}  J={J:.2f}  "
                      f"rms {t.rms_error*1000:.2f} mrad  tau máx {t.max_tau:.2f} Nm")

    return informe(todos, posturas, articulaciones, grids)



def curvas(todos, posturas, articulaciones, grids):
    """Imprime J(kp) por postura. Cuando el argmin topa, la curva es el dato."""
    for idx in articulaciones:
        print(f"\n  ── J(kp) en {BY_INDEX[idx].name} "
              f"{'─' * max(0, 40 - len(BY_INDEX[idx].name))}")
        print(f"  {'kp':>8}" + "".join(f"{p:>12}" for p in posturas))
        kps = sorted({kp for kp, _ in grids.get(idx, [])})
        for kp in kps:
            fila = f"  {kp:>8.1f}"
            for p in posturas:
                r = todos.get((p, idx), [])
                v = next((J for J, k, _, _, _ in r if abs(k - kp) < 1e-6), None)
                fila += f"{v:>12.2f}" if v is not None else f"{'—':>12}"
            print(fila)


def aviso_borde_inferior(articulaciones):
    """El borde de ABAJO no es el tope de saturación: ahí sí se puede ampliar.

    Distinguirlos importa. Si el ganador topa arriba, el borde es el límite de
    par y no hay nada que hacer salvo cambiar de criterio. Si topa abajo, el
    límite es la rejilla y el óptimo está fuera: el ensayo simplemente no ha
    encontrado el mínimo y hay que repetirlo con kp más bajos.
    """
    print(f"""
  ‼ {', '.join(BY_INDEX[i].name for i in articulaciones)} topó por ABAJO.

  Su J crece de forma monótona con kp: el mínimo está por debajo del kp más
  bajo que se probó, así que el óptimo NO se ha medido. Aquí el borde no es un
  límite físico —la saturación está arriba— sino de la rejilla, y se arregla
  repitiendo con valores más bajos.

  Esta articulación queda FUERA de la decisión: no hay kp* que comparar.""")


def criterio_secundario(todos, posturas, articulaciones, holgura=0.10):
    """kp SUFICIENTE: el más bajo cuyo J queda a menos de `holgura` del mínimo.

    ─────────────────────────────────────────────────────────────────────────
    ESTA REGLA SE AÑADIÓ EL 2026-09-10, DESPUÉS DE VER DATOS. Hay que decirlo.
    ─────────────────────────────────────────────────────────────────────────

    La regla de la cabecera compara `kp*` entre posturas, y da por supuesto que
    `kp*` es un mínimo INTERIOR. Medido: no lo es. En el codo, J baja de forma
    monótona hasta el tope de saturación en P1 y en P3, así que `kp*` es el
    tope en las dos y la razón sale 1.00 por construcción. La regla primaria no
    es que salga negativa: es que **no aplica**.

    Lo que sigue siendo medible cuando el argmin topa es la FORMA de la curva.
    `kp_suficiente` es el codo de J(kp): por debajo se paga error, por encima
    ya casi no se gana. Si la postura cambia la planta, cambia dónde está ese
    codo, y eso sí se ve aunque los dos argmin estén pegados al tope.

    Se le aplica el mismo umbral de 1.5 que a la regla primaria, para no
    inventar también el umbral después de ver los datos.
    """
    print(f"""
  ‼ La regla primaria NO APLICA en {', '.join(BY_INDEX[i].name for i in articulaciones)}.

  Su `kp*` cayó en el borde de la rejilla en TODAS las posturas, así que la
  razón vale 1.00 porque el barrido topó, no porque las posturas coincidan.
  Como el borde superior es el tope de saturación —y no se puede subir sin
  salirse de la zona segura— no se arregla ampliando el rango.

  Criterio secundario, sobre el codo de la curva J(kp): el kp más bajo cuyo J
  queda a menos del {holgura*100:.0f} % del mínimo. Mismo umbral de {RAZON_UMBRAL}.""")
    out = {}
    print(f"\n  {'articulación':<20}" + "".join(f"{p:>12}" for p in posturas)
          + f"{'razón':>9}")
    for idx in articulaciones:
        fila = f"  {BY_INDEX[idx].name:<20}"
        sufs = []
        for p in posturas:
            r = todos.get((p, idx), [])
            if not r:
                fila += f"{'—':>12}"; continue
            Jmin = min(J for J, *_ in r)
            cand = [k for J, k, *_ in r if J <= Jmin * (1.0 + holgura)]
            suf = min(cand) if cand else None
            sufs.append(suf)
            fila += f"{suf:>12.1f}"
        if len(sufs) >= 2 and all(s for s in sufs):
            rr = max(sufs) / min(sufs)
            out[idx] = rr
            fila += f"{rr:>8.2f}" + ("  ⚠" if rr >= RAZON_UMBRAL else "")
        print(fila)
    print(f"\n  (kp suficiente, en las mismas unidades que kp*)")
    return out


def informe(todos, posturas, articulaciones, grids) -> int:
    if not todos:
        print("\n  sin resultados.")
        return 1
    ganadores = {k: v[0] for k, v in todos.items()}
    print("\n\n  ══ F3 · resultados ══════════════════════════════════════════")
    print(f"\n  {'articulación':<20}" + "".join(f"{p:>22}" for p in posturas)
          + f"{'razón':>9}")
    print(f"  {'':<20}" + "".join(f"{'kp*  kd*  rms  tau':>22}" for _ in posturas))
    razones, en_borde = {}, set()
    por_arriba, por_abajo = set(), set()
    # El borde de la rejilla REAL, no el de la lista pedida: `rejilla_de`
    # recorta por saturación, así que casi nunca coinciden.
    bordes = {i: ({min(kp for kp, _ in grids[i]), max(kp for kp, _ in grids[i])}
                  if grids.get(i) else set()) for i in articulaciones}
    for idx in articulaciones:
        fila = f"  {BY_INDEX[idx].name:<20}"
        kps = []
        for p in posturas:
            g = ganadores.get((p, idx))
            if g is None:
                fila += f"{'—':>22}"
                continue
            J, kp, kd, t, st = g
            kps.append(kp)
            fila += (f"{kp:>6.0f}{kd:>5.1f}{t.rms_error*1000:>6.1f}"
                     f"{t.max_tau:>5.1f}")
        if len(kps) >= 2:
            r = max(kps) / min(kps)
            razones[idx] = r
            fila += f"{r:>8.2f}" + ("  ⚠" if r >= RAZON_UMBRAL else "")
        if kps and bordes.get(idx) and all(k in bordes[idx] for k in kps):
            lo = min(kp for kp, _ in grids[idx])
            arriba = all(abs(k - lo) > 1e-6 for k in kps)
            fila += ("  ‼ todos en el borde SUPERIOR" if arriba
                     else "  ‼ todos en el borde INFERIOR")
            en_borde.add(idx)
            (por_arriba if arriba else por_abajo).add(idx)
        print(fila)
    print(f"\n  (kp*, kd*, error rms en mrad, par máximo en Nm)")

    if not razones:
        print("\n  Una sola postura: no hay razón que calcular. Para decidir "
              "hacen falta\n  al menos dos, y la comparación que importa es "
              "P1 contra P3.")
        return 0
    secundarias = set()
    if en_borde:
        curvas(todos, posturas, sorted(en_borde), grids)
        if por_abajo:
            aviso_borde_inferior(sorted(por_abajo))
        if por_arriba:
            razones_suf = criterio_secundario(todos, posturas, sorted(por_arriba))
        else:
            razones_suf = {}
        for i in por_abajo:
            razones.pop(i, None)
        razones.update(razones_suf)
        secundarias = set(razones_suf)

    if not razones:
        print("""
  SIN DECISIÓN.

  Ninguna articulación deja una razón que comparar: todas toparon por abajo,
  o sea que el mínimo está fuera de la rejilla. Repite con kp más bajos antes
  de concluir nada.""")
        return 1

    peor = max(razones.values())
    quien = BY_INDEX[max(razones, key=razones.get)].name
    print(f"\n  Razón mayor: {peor:.2f} en {quien}  (umbral {RAZON_UMBRAL})")
    if peor < RAZON_UMBRAL:
        nota = ("  (en las marcadas ‼ la comparación es del kp suficiente, "
                "no del kp*)\n" if secundarias else "")
        print(f"""
  DECISIÓN: un solo conjunto de ganancias.

{nota}  Ninguna articulación cambia su kp en más de un factor {RAZON_UMBRAL}
  entre la postura de reposo y la extendida. Se elige el de P3, que es el peor
  caso de inercia y gravedad, y se documenta cuánto se degrada en P1 y P2.""")
    else:
        magnitud = ("su kp suficiente —el codo de J(kp)—"
                    if max(razones, key=razones.get) in secundarias
                    else "su kp óptimo")
        print(f"""
  DECISIÓN: gain scheduling por postura.

  {quien} cambia {magnitud} por un factor {peor:.2f}, o sea que una
  sintonización única no preserva el margen de fase en todo el espacio de
  trabajo. Se implementa como tabla de tres puntos interpolada linealmente,
  con kp(q) proporcional a M_ii(q) del mismo modelo de `pinocchio`.""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
