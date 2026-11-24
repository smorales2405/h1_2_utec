#!/usr/bin/env python3
"""Núcleo del regulador de fuerza: lectura, seguridad, detectores y bitácora.

Todo lo que el lazo necesita por debajo del control, en un solo sitio, porque
A2–A5 y la Parte B lo comparten. **Aquí no hay controlador**: el control vive en
`lazo.py` y usa esto como plataforma.

Decisiones que vienen medidas del Exp 3, no elegidas:

- **Frescura POR DOF.** La mano publica estado cada ~30.7 ms se lea a la tasa que
  se lea. `Caracterizacion/exp3/exp3_min_step.py` ya marcaba frescura, pero **por
  bloque** («algún DOF cambió»), que no sirve para un lazo multi-dedo. Integrar
  sobre duplicados infla `Ki` en la proporción entre tasa de sondeo y de refresco
  —del orden de 18×— así que el control solo puede avanzar con muestra fresca del
  dedo que regula.
- **Dos detectores de resbalón, no uno.** Los dos se ven como caída de fuerza y la
  reacción correcta es **opuesta**:

  |  | `POS` | Fuerza | Reacción |
  |---|---|---|---|
  | Resbalón del ACTUADOR | retrocede contra su comando | cae | **aflojar**: ya se pasó del borde |
  | Objeto que SE ESCAPA | sigue al comando | cae en **todos** los dedos a la vez | **apretar** o abortar |

  `POS` es lo único que los separa. Medido: el resbalón del actuador mueve `POS`
  ~50 counts en menos de medio segundo, contra 0–2 counts del sostenimiento
  normal (E3.4), así que un umbral de 10 counts es inequívoco.
- **Sin tope de fuerza constante.** El borde depende de la pose: 455–745 g en lo
  medido. La guarda de fuerza sigue existiendo como red de seguridad, pero quien
  encuentra el borde es el detector de resbalón.
- **Tara solo con la mano abierta, descargada y ≥ 10 s después de soltar.** Tras
  sostener carga el cero queda corrido 21–46 g y tarda ~8 s en volver (E3.5), y el
  signo es **por DOF**: índice −46 g, pulgar +21 g.
- **`FORCE_SET` siempre por encima del rango de trabajo.** Si `FORCE_ACT` lo
  alcanza, el dedo deja de aceptar `ANGLE_SET` **en los dos sentidos**: el paro del
  firmware secuestra la posición.
"""
from __future__ import annotations

import csv
import os
import statistics
import sys
import time
from collections import deque

_HERE = os.path.dirname(os.path.abspath(__file__))
_CARACT = os.path.join(os.path.dirname(_HERE), 'Caracterizacion')
sys.path.insert(0, _CARACT)

from hand_modbus import (                                    # noqa: E402
    HandModbus, NDOF, ANGLE_SET, ANGLE_ACT, FORCE_SET, SPEED_SET,
    POS_ACT, FORCE_ACT, CURRENT, DOF_NAMES, open_vector, report_hold,
)

FORCE_CLB = 1009        # tara de fuerza; escribir 1 con la palma abierta y descargada
FSET_ABRIR = 3000       # FORCE_SET antes de abrir: nunca debe frenar la apertura

# POS crece al FLEXIONAR y ANGLE_SET decrece al flexionar. Un dedo que "retrocede"
# es POS bajando sin que nadie se lo haya pedido.
CIERRA_POS = +1


def vector(cmds, hold):
    """Bloque ANGLE_SET con varios DOF a la vez; el resto en -1 (no tocar)."""
    v = [-1] * NDOF
    for d, a in (hold or {}).items():
        v[d] = int(a)
    for d, a in cmds.items():
        v[d] = int(a)
    return v


def conectar(args):
    return HandModbus.open_tcp(args.ip, args.port, args.device_id, args.timeout)


# ── lectura con frescura por DOF ──────────────────────────────────────────
class Lector:
    """Sondea POS/FORCE (y CURRENT cada `aux_every`) y devuelve DOS niveles de
    novedad, porque hacen falta los dos.

    - **`frame`**: el bloque entero cambió → llegó estado nuevo de la mano. Es lo
      que debe disparar el control. Fiable, porque con seis DOF algo casi siempre
      cambia.
    - **`fresca_f[d]` / `fresca_p[d]`**: ese DOF concreto trae valor nuevo. Dice si
      el dedo aporta información, pero **subestima**: si el entero se repite entre
      dos frames cuenta como no fresco, y con ruido de ±1–2 g eso pasa en torno a
      una de cada cuatro veces. Medido contra el mock: 26–27 Hz por DOF frente a
      32.6 Hz de frames reales.

    Para `POS` la subestimación es total y esperada: con un solo dedo en
    movimiento, el `POS` de los demás no cambia nunca (E3.6a).
    """

    def __init__(self, hand, aux_every=8):
        self.h = hand
        self.aux_every = aux_every
        self.prev_p = [None] * NDOF
        self.prev_f = [None] * NDOF
        self._prev_bloque_p = None
        self._prev_bloque_f = None
        self.cur = [None] * NDOF
        self.i = 0
        self.n_lecturas = 0
        self.n_frames = 0
        self.n_frescas_f = [0] * NDOF
        self.n_frescas_p = [0] * NDOF
        self.t_ini = None
        self._dt = deque(maxlen=400)
        self._t_ant = None

    def muestra(self):
        """Devuelve (t, pos, fuerza, corriente, fresca_p, fresca_f)."""
        p = self.h.read_block(POS_ACT)
        f = self.h.read_block(FORCE_ACT)
        t = time.perf_counter()
        if self.t_ini is None:
            self.t_ini = t
        if self.i % self.aux_every == 0:
            c = self.h.read_block(CURRENT)
            if c:
                self.cur = c
        self.i += 1
        self.n_lecturas += 1
        if self._t_ant is not None:
            self._dt.append(t - self._t_ant)
        self._t_ant = t

        # "Frame" de bloque: SOBRECUENTA y es solo diagnostico. Se penso como la
        # deteccion fiable de "llego estado nuevo", suponiendo que la mano publica
        # los 6 DOF a la vez. **No lo hace**: E3.6a midio 1.9 ms de desfase entre
        # indice y pulgar, asi que el bloque cambia cada vez que se actualiza
        # CUALQUIERA de los seis y la tasa sale inflada (51 Hz medidos contra 32.6
        # de publicacion). El disparo del control va por DOF, con `Disparador`.
        frame = (p is not None and p != self._prev_bloque_p) or \
                (f is not None and f != self._prev_bloque_f)
        if p is not None:
            self._prev_bloque_p = list(p)
        if f is not None:
            self._prev_bloque_f = list(f)
        self.n_frames += int(frame)

        fp = [False] * NDOF
        ff = [False] * NDOF
        if p:
            for d in range(NDOF):
                if self.prev_p[d] is None or p[d] != self.prev_p[d]:
                    fp[d] = self.prev_p[d] is not None
                    self.prev_p[d] = p[d]
            for d in range(NDOF):
                self.n_frescas_p[d] += int(fp[d])
        if f:
            for d in range(NDOF):
                if self.prev_f[d] is None or f[d] != self.prev_f[d]:
                    ff[d] = self.prev_f[d] is not None
                    self.prev_f[d] = f[d]
            for d in range(NDOF):
                self.n_frescas_f[d] += int(ff[d])
        return t, p, f, list(self.cur), fp, ff, frame

    def tasas(self, dofs):
        """(Hz sondeo, Hz de frame, {dof: Hz fuerza fresca}, {dof: Hz POS fresca}, jitter ms)."""
        if self.t_ini is None or self.n_lecturas < 2:
            return 0.0, 0.0, {}, {}, 0.0
        T = time.perf_counter() - self.t_ini
        if T <= 0:
            return 0.0, 0.0, {}, {}, 0.0
        dts = sorted(self._dt)
        jit = 0.0
        if len(dts) > 10:
            jit = (dts[int(len(dts) * 0.9)] - dts[int(len(dts) * 0.1)]) * 1000
        return (self.n_lecturas / T, self.n_frames / T,
                {d: self.n_frescas_f[d] / T for d in dofs},
                {d: self.n_frescas_p[d] / T for d in dofs},
                jit)


class Disparador:
    """Decide cuándo el control puede avanzar para un DOF concreto.

    Medido en banco, y con la mano publicando cada 30.7 ms por DOF:

    | | cambios de fuerza | intervalo mediano |
    |---|---|---|
    | sin carga | 6–11 Hz | **60.0 ms** = 2 × 30.7 |
    | con carga (330–1077 g) | **28.3 Hz** | **30.4 ms** |

    Los intervalos son múltiplos exactos del periodo: el dedo publica siempre, pero
    si el entero se repite la lectura parece «no fresca». Bajo carga —el régimen
    del regulador— apenas ocurre; sin carga pasa uno de cada dos frames.

    Por eso el disparo es **cambio de valor O tiempo agotado**: el cambio da la
    cadencia natural de 30 ms, y el plazo evita que el lazo se quede parado cuando
    el valor se repite. Un valor repetido no es información vieja: es el valor
    actual. Lo que no puede hacerse es integrar VARIAS veces dentro del mismo
    frame, que es lo que inflaría `Ki`.
    """

    def __init__(self, dof, plazo_s=0.040):
        self.dof, self.plazo = dof, plazo_s
        self.t_ultimo = None
        self.n_cambio = 0
        self.n_plazo = 0

    def toca(self, t, fresca_f):
        if self.t_ultimo is None:
            self.t_ultimo = t
            return True, 0.0, 'inicio'
        dt = t - self.t_ultimo
        if fresca_f[self.dof]:
            self.t_ultimo = t
            self.n_cambio += 1
            return True, dt, 'cambio'
        if dt >= self.plazo:
            self.t_ultimo = t
            self.n_plazo += 1
            return True, dt, 'plazo'
        return False, dt, ''

    def resumen(self, T):
        n = self.n_cambio + self.n_plazo
        return (f"{n/T:.1f} Hz de disparo · {self.n_cambio} por cambio, "
                f"{self.n_plazo} por plazo ({100*self.n_plazo/max(n,1):.0f} %)")


# ── seguridad ─────────────────────────────────────────────────────────────
class Guarda:
    """Red de seguridad: fuerza, corriente sostenida, temperatura y tiempo.

    NO incluye un tope de fuerza «de diseño»: el borde de lo sostenible depende de
    la pose (455–745 g medidos) y quien lo encuentra es `DetectorResbalon`. Esta
    guarda es el último recurso, y su techo debe quedar **por debajo** de
    `FORCE_SET` para que el software actúe antes que el paro del firmware.
    """

    def __init__(self, args, dofs):
        self.a, self.dofs = args, dofs
        self.motivo = None
        self.cur_alta = 0
        self.f_alta = 0
        self.t0 = time.perf_counter()

    def revisa(self, f, c, temps=None):
        a = self.a
        # Exige PERSISTENCIA, como ya hacia la corriente. Una sola lectura
        # corrupta basta para abortar: en la tanda de perturbacion la guarda
        # salto con 3187 g —imposible para este sensor— y el final abrupto dejo
        # carga en la yema, que ademas hizo rechazar la tara del trial siguiente.
        # A ~400 Hz, 3 muestras son <10 ms: no compromete la seguridad.
        if f:
            alto = [d for d in self.dofs if abs(f[d]) > a.techo_fuerza]
            if alto:
                self.f_alta += 1
                if self.f_alta >= a.fuerza_ciclos:
                    d = alto[0]
                    self.motivo = (f"{DOF_NAMES[d]}: {f[d]} g > techo {a.techo_fuerza} "
                                   f"en {self.f_alta} muestras seguidas")
                    return self.motivo
            else:
                self.f_alta = 0
        if c and any(c[d] is not None and c[d] > a.corriente_max for d in self.dofs):
            self.cur_alta += 1
            if self.cur_alta > a.corriente_ciclos:
                self.motivo = f"corriente > {a.corriente_max} mA sostenida"
                return self.motivo
        else:
            self.cur_alta = 0
        if temps and max(temps[d] for d in self.dofs) > a.temp_max:
            self.motivo = f"temperatura {max(temps[d] for d in self.dofs)} °C"
            return self.motivo
        if time.perf_counter() - self.t0 > a.timeout_s:
            self.motivo = f"timeout de {a.timeout_s:.0f} s"
            return self.motivo
        return None


def abrir_mano(hand, args, hold=None):
    """Apertura segura: primero se quita el paro del firmware, luego se abre.

    El orden importa: un dedo que haya llegado a `FORCE_SET` no acepta `ANGLE_SET`
    en ningún sentido hasta que el umbral suba.
    """
    try:
        hand.write_block(FORCE_SET, [FSET_ABRIR] * NDOF)
        hand.write_block(SPEED_SET, [args.vel_abrir] * NDOF)
        hand.write_block(ANGLE_SET, open_vector(args.angulo_abierto, hold or {}))
        time.sleep(0.4)
        return True
    except Exception:
        return False


# ── detectores de resbalón ────────────────────────────────────────────────
class DetectorResbalon:
    """Resbalón del ACTUADOR: `POS` retrocede contra su propio comando.

    Medido: al pasarse del borde, el dedo retrocede ~49 counts en menos de medio
    segundo y la fuerza cae 553 g; en sostenimiento normal `POS` se mueve 0–2
    counts en 60 s (E3.4). Entre 2 y 50 hay un abismo, así que 10 counts en 0.5 s
    es inequívoco.

    Un objeto que se mueve **no** puede empujar al dedo contra su comando: si `POS`
    retrocede, el que cede es el mecanismo. Ésa es la firma que lo distingue del
    objeto escapándose, y la reacción correcta es **aflojar**, no apretar.
    """

    def __init__(self, dofs, counts=10, ventana_s=0.5):
        self.dofs, self.counts, self.ventana = dofs, counts, ventana_s
        self.hist = {d: deque() for d in dofs}          # (t, pos)
        self.cmd_hist = {d: deque() for d in dofs}      # (t, comando)
        self.eventos = []

    def reinicia(self):
        """Olvida la historia. Entre trials la mano se abre, y ese recorrido de
        POS leido a traves de la ventana parece un retroceso contra el comando."""
        for d in self.dofs:
            self.hist[d].clear(); self.cmd_hist[d].clear()

    def actualiza(self, t, p, cmds, f=None, minimo=0.0):
        # ARMADO SOLO EN CONTACTO. La firma que busca este detector —POS retrocede
        # contra su propio comando— la cumple trivialmente cualquier dedo que aun
        # venga viajando del comando anterior. Sin carga no hay nada de lo que
        # protegerse: el actuador no puede ceder contra una fuerza que no existe.
        # Mientras no haya contacto se olvida la historia, para que la ventana
        # entre limpia en cuanto lo haya.
        if f is not None and minimo > 0 and max(abs(f[d]) for d in self.dofs) < minimo:
            for d in self.dofs:
                self.hist[d].clear(); self.cmd_hist[d].clear()
            return None
        if not p:
            return None
        for d in self.dofs:
            self.hist[d].append((t, p[d]))
            self.cmd_hist[d].append((t, cmds.get(d)))
            while self.hist[d] and t - self.hist[d][0][0] > self.ventana:
                self.hist[d].popleft()
            while self.cmd_hist[d] and t - self.cmd_hist[d][0][0] > self.ventana:
                self.cmd_hist[d].popleft()
            if len(self.hist[d]) < 5:
                continue
            # MEDIANA de los extremos, no la muestra suelta de cada punta. Una
            # sola lectura mala basta para fabricar un resbalon: en la compuerta
            # A2 una muestra aislada dio POS 988 entre vecinas de 851 y aborto un
            # trial de 60 s. Un resbalon real dura ~0.5 s y ~50 counts, asi que
            # sobrevive de sobra a promediar las puntas.
            k = max(1, min(5, len(self.hist[d]) // 2))
            vals = [v for _, v in self.hist[d]]
            retroceso = statistics.median(vals[:k]) - statistics.median(vals[-k:])
            if retroceso * CIERRA_POS < self.counts:
                continue
            # ¿se le pidió abrir EN ALGUN MOMENTO de la ventana? entonces no es
            # resbalon, es obediencia con retardo. Comparar solo el primer comando
            # con el ultimo no basta: si el lazo abre y revierte dentro de la
            # ventana, el neto es cero y el POS bajando parece un resbalon. Paso
            # en el cubo de PLA —cmd 702→708→702 en 0.28 s— y aborto la tanda.
            cmds_v = [c for _, c in self.cmd_hist[d] if c is not None]
            if len(cmds_v) >= 2 and max(cmds_v) - cmds_v[0] >= 2:    # ANGLE_SET subio = abrir
                continue
            ev = dict(t=t, dof=d, counts=retroceso, ventana=self.ventana)
            self.eventos.append(ev)
            self.hist[d].clear()
            return ev
        return None


class DetectorEscape:
    """El OBJETO se escapa: la fuerza cae en TODOS los dedos a la vez y `POS` no
    retrocede.

    Es el complementario del anterior y se confunden con él si solo se mira la
    fuerza. Aquí el mecanismo no cede —`POS` sigue al comando— pero el objeto deja
    de empujar contra las yemas. La reacción es **apretar** o abortar, es decir la
    contraria.
    """

    def __init__(self, dofs, caida_g=60.0, frac=0.35, ventana_s=0.5, pos_counts=6):
        self.dofs, self.ventana = dofs, ventana_s
        self.caida, self.frac, self.pos_counts = caida_g, frac, pos_counts
        self.hist = {d: deque() for d in dofs}          # (t, fuerza, pos)
        self.eventos = []

    def reinicia(self):
        for d in self.dofs:
            self.hist[d].clear()

    def actualiza(self, t, p, f):
        if not (p and f):
            return None
        for d in self.dofs:
            self.hist[d].append((t, f[d], p[d]))
            while self.hist[d] and t - self.hist[d][0][0] > self.ventana:
                self.hist[d].popleft()
        if any(len(self.hist[d]) < 5 for d in self.dofs):
            return None
        caidas, pos_ok = [], True
        for d in self.dofs:
            f0, f1 = self.hist[d][0][1], self.hist[d][-1][1]
            p0, p1 = self.hist[d][0][2], self.hist[d][-1][2]
            caidas.append(f0 - f1)
            if (p0 - p1) * CIERRA_POS >= self.pos_counts:
                pos_ok = False                       # eso es resbalón del actuador
        if not pos_ok:
            return None
        base = [self.hist[d][0][1] for d in self.dofs]
        if all(c > self.caida for c in caidas) and \
           all(c > self.frac * max(b, 1) for c, b in zip(caidas, base)):
            ev = dict(t=t, caidas={d: c for d, c in zip(self.dofs, caidas)})
            self.eventos.append(ev)
            for d in self.dofs:
                self.hist[d].clear()
            return ev
        return None


# ── política de tara ──────────────────────────────────────────────────────
class Tara:
    """Tara de fuerza con las condiciones que E3.5 impone.

    Tras sostener carga el cero queda corrido 21–46 g y vuelve a +6 g/s, o sea ~8 s.
    Tarar antes mete ese error directamente en la consigna del lazo. Por eso la
    tara exige mano abierta, descargada y **≥ 10 s desde la última suelta**.
    """

    def __init__(self, espera_s=10.0, umbral_g=40.0):
        self.espera, self.umbral = espera_s, umbral_g
        self.t_suelta = None
        self.n = 0

    def marca_suelta(self):
        self.t_suelta = time.perf_counter()

    def puede(self, f, dofs):
        if self.t_suelta is not None:
            falta = self.espera - (time.perf_counter() - self.t_suelta)
            if falta > 0:
                return False, f"faltan {falta:.0f} s desde la última suelta (E3.5)"
        # Solo cuenta la carga POSITIVA. Una yema en contacto empuja, y eso lee
        # positivo; un valor negativo no puede ser contacto, es el cero corrido
        # —justo lo que la tara existe para quitar—. Con `abs()` la politica se
        # mordia la cola: el indice a -45 g bloqueaba la tara que lo corregia.
        if f and any(f[d] > self.umbral for d in dofs):
            cargados = [f"{DOF_NAMES[d]} {f[d]} g" for d in dofs if f[d] > self.umbral]
            return False, f"hay carga en las yemas: {', '.join(cargados)}"
        return True, "ok"

    def aplica(self, hand, f, dofs, forzar=False):
        ok, por_que = self.puede(f, dofs)
        if not ok and not forzar:
            return False, por_que
        hand.write_block(FORCE_CLB, [1])
        time.sleep(1.5)
        self.n += 1
        return True, "tarada"


class EstimadorRLS:
    """Minimos cuadrados recursivos de la matriz de ganancia de la pinza.

        [ΔF_T]   [k_TT  k_TI] [Δpos_T]
        [ΔF_I] = [k_IT  k_II] [Δpos_I]

    **Contra `Δpos`, no contra `Δcmd`.** Un comando que no se ejecuta —holgura,
    saturacion, el dedo aun viajando— haria concluir ganancia cero justo cuando
    la ganancia es alta. `POS` dice lo que el dedo hizo de verdad.

    **Sin escalones de sondeo**, como pide el plan: se aprende de las acciones del
    propio lazo. El precio es que la excitacion no esta garantizada, y de ahi las
    dos protecciones:

    - **Congelar con poco movimiento.** Si `Σ|Δpos| < min_pos`, la muestra es
      ruido dividido por casi cero. Medido en banco: el 15 % de las acciones dan
      `Δpos = 0` y el 38 % menos de 3 counts.
    - **Regularizacion hacia el prior.** Si los dos dedos se mueven siempre en la
      misma proporcion, las columnas son colineales y las cruzadas NO son
      identificables. El ridge mantiene la estimacion pegada a la `J` medida en
      banco en vez de dejarla irse por un modo no excitado.
    """

    def __init__(self, prior, lam=0.98, min_pos=5.0, ridge=1e-3, p0=10.0,
                 ridge_cruz=None):
        self.K = [list(prior[0]), list(prior[1])]      # [[kTT,kTI],[kIT,kII]]
        self.prior = [list(prior[0]), list(prior[1])]
        self.lam, self.min_pos, self.ridge = lam, min_pos, ridge
        # RIDGE ANISOTROPO. Replayando A3, la correlacion entre Δpos_pulgar y
        # Δpos_indice sale -0.84: el lazo casi siempre abre uno y cierra el otro,
        # asi que las columnas son casi colineales y las CRUZADAS apenas son
        # identificables (la estimacion de k_TI cayo de 5.43 a ~1.0 con 16
        # muestras). La diagonal si esta bien excitada. Por eso las cruzadas se
        # atan al prior mucho mas fuerte que la diagonal.
        self.ridge_cruz = ridge * 10 if ridge_cruz is None else ridge_cruz
        self.P = [[p0, 0.0], [0.0, p0]]                # covarianza compartida
        # Umbral de singularidad RELATIVO al prior, no absoluto: `det` escala con
        # el cuadrado de las ganancias, asi que un numero fijo vale para un objeto
        # y no para otro. Con la mano vacia el det se hundio a 0.08 y un umbral de
        # 1e-3 lo dejo pasar; lo unico que evito el disparate fue el recorte por
        # dq_max, que es una red, no un criterio.
        self.det_min = 0.05 * abs(prior[0][0] * prior[1][1] - prior[0][1] * prior[1][0])
        self.n_uso = 0
        self.n_congelado = 0

    def actualiza(self, dpos, dF):
        """Una accion: dpos=(ΔPOS_T, ΔPOS_I), dF=(ΔF_T, ΔF_I)."""
        x0, x1 = dpos
        if abs(x0) + abs(x1) < self.min_pos:
            self.n_congelado += 1
            return False
        P, lam = self.P, self.lam
        # ganancia de Kalman: g = P x / (lam + x' P x)
        Px = (P[0][0] * x0 + P[0][1] * x1, P[1][0] * x0 + P[1][1] * x1)
        den = lam + x0 * Px[0] + x1 * Px[1]
        if den <= 1e-9:
            return False
        g = (Px[0] / den, Px[1] / den)
        for i in range(2):                              # una fila por dedo
            pred = self.K[i][0] * x0 + self.K[i][1] * x1
            err = dF[i] - pred
            for j in range(2):
                self.K[i][j] += g[j] * err
                r = self.ridge if i == j else self.ridge_cruz
                self.K[i][j] += r * (self.prior[i][j] - self.K[i][j])
        for i in range(2):
            for j in range(2):
                P[i][j] = (P[i][j] - g[i] * Px[j]) / lam
        self.n_uso += 1
        return True

    def banda(self, i, paso, ratio, k=0.5, minimo=8.0):
        """Banda muerta dimensionada por la ganancia ESTIMADA, no por constante.

        Si el menor cambio de fuerza que el dedo puede producir es
        `paso × K_ii × ratio`, el mejor error que puede garantizar es la MITAD de
        eso: siempre se puede quedar a medio escalon. Pedirle menos es pedirle que
        oscile. Y como `K` es funcion del objeto y del punto de trabajo, la banda
        tambien tiene que serlo.

        Medido: el mismo controlador que sobre la espuma quiere ~24 g, sobre el
        cubo de PLA necesita ~60 —ahi el indice movia 148 g con 6 unidades— y con
        la banda fija oscilaba. Esto es el *scheduling continuo* que pide el plan,
        aplicado a la banda en vez de a la ganancia del PI.

        El suelo evita que una ganancia estimada baja deje la banda por debajo del
        ruido de fuerza (±1-2 g medidos).
        """
        return max(minimo, k * paso * abs(self.K[i][i]) * abs(ratio))

    def bandas_coord(self, paso, rT, rI, k=0.5, minimo=4.0):
        """Bandas muertas en las coordenadas que importan: (apriete, balance).

        Dimensionar POR DEDO es demasiado conservador, y esta medido: el balance
        no es tarea de un solo dedo —`J^-1` lo reparte y el pulgar fino carga la
        precision que el indice basto no da—, asi que **la pareja puede hacerlo
        mejor que el peor de sus dedos**. Sizar la banda del indice por su propio
        cuanto dio 70 g y rompio el seguimiento del balance; con la banda fija de
        24 g el mismo lazo lo hacia bien.

        Lo correcto es el menor cambio que el PAR puede producir en cada
        coordenada, que es el minimo sobre las acciones minimas disponibles:

            mover solo el pulgar   → ds = ½(K_TT + K_IT)·rT·paso
                                     db =  (K_TT − K_IT)·rT·paso
            mover solo el indice   → ds = ½(K_TI + K_II)·rI·paso
                                     db =  (K_TI − K_II)·rI·paso

        Con la `K` de la bola eso da ~4 g de banda de balance moviendo solo el
        pulgar, contra los 70 del indice: un factor 17.
        """
        dsT = 0.5 * (self.K[0][0] + self.K[1][0]) * abs(rT) * paso
        dbT = (self.K[0][0] - self.K[1][0]) * abs(rT) * paso
        dsI = 0.5 * (self.K[0][1] + self.K[1][1]) * abs(rI) * paso
        dbI = (self.K[0][1] - self.K[1][1]) * abs(rI) * paso
        ds = min(abs(dsT), abs(dsI))
        db = min(abs(dbT), abs(dbI))
        return (max(minimo, k * ds), max(minimo, k * db))

    def det(self):
        return self.K[0][0] * self.K[1][1] - self.K[0][1] * self.K[1][0]

    def sano(self):
        """Una diagonal no positiva es fisicamente imposible —cerrar un dedo no
        puede bajar su propia fuerza— y significa que la estimacion se fue."""
        return self.K[0][0] > 0.1 and self.K[1][1] > 0.1

    def inversa(self, det_min=None):
        """J^-1, o None si esta mal condicionada: invertir algo casi singular
        manda correcciones enormes en la direccion equivocada."""
        d = self.det()
        if abs(d) < (self.det_min if det_min is None else det_min) or not self.sano():
            return None
        return ((self.K[1][1] / d, -self.K[0][1] / d),
                (-self.K[1][0] / d, self.K[0][0] / d))

    def __str__(self):
        return (f"[{self.K[0][0]:6.2f} {self.K[0][1]:6.2f} ; "
                f"{self.K[1][0]:6.2f} {self.K[1][1]:6.2f}]  det={self.det():7.2f}")


# ── bitácora ──────────────────────────────────────────────────────────────
class Bitacora:
    """CSV por tanda, escrito al vuelo (el vigilante de memoria mata procesos que
    acumulan). Las columnas de control quedan vacías mientras no haya controlador."""

    COLS = ['t', 'frame', 'fresca_f', 'fresca_p', 'F_T', 'F_I', 'F_M', 'F_grip',
            'err_s', 'err_b', 'u_s', 'u_t', 'dq_T', 'dq_I', 'dq_M',
            'POS_T', 'POS_I', 'POS_M', 'cmd_T', 'cmd_I', 'cmd_M',
            'I_mA', 'temp', 'evento']

    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.fh = open(path, 'w', newline='')
        self.w = csv.writer(self.fh)
        self.w.writerow(self.COLS)
        self.path = path
        self.n = 0

    def fila(self, **kw):
        self.w.writerow([kw.get(c, '') for c in self.COLS])
        self.n += 1
        if self.n % 200 == 0:
            self.fh.flush()

    def cierra(self):
        self.fh.flush()
        self.fh.close()


def argumentos_comunes(p):
    """Los que comparten todas las fases del regulador."""
    p.add_argument('--ip', default='192.168.124.210')
    p.add_argument('--port', type=int, default=6000)
    p.add_argument('--device-id', type=int, default=1)
    p.add_argument('--timeout', type=float, default=1.0)
    p.add_argument('--rot', type=int, default=0,
                   help='ANGLE_SET de la rotación del pulgar. Obligatorio anclarla en todo '
                        'lo que toque el PULGAR: sin ello describe otra trayectoria. Usa -1 '
                        'para NO anclar, que es como se caracterizó el índice solo (E3.1-E3.5)')
    p.add_argument('--aux-every', type=int, default=8, help='cada cuántas lecturas se lee CURRENT')
    p.add_argument('--vel-cierre', type=int, default=25)
    p.add_argument('--vel-abrir', type=int, default=300)
    p.add_argument('--angulo-abierto', type=int, default=1000)
    # seguridad
    p.add_argument('--techo-fuerza', type=float, default=600.0,
                   help='red de seguridad, NO el tope de diseño. Debe quedar por debajo '
                        'de --fset-respaldo')
    p.add_argument('--fset-respaldo', type=int, default=700,
                   help='paro del firmware; por encima del techo software')
    p.add_argument('--corriente-max', type=int, default=700)
    p.add_argument('--fuerza-ciclos', type=int, default=3,
                   help='muestras seguidas sobre el techo antes de abortar; '
                        'una lectura corrupta suelta no debe parar una tanda')
    p.add_argument('--corriente-ciclos', type=int, default=25)
    p.add_argument('--temp-max', type=int, default=50)
    p.add_argument('--timeout-s', type=float, default=600.0)
    # detectores
    p.add_argument('--resbalon-counts', type=int, default=10,
                   help='retroceso de POS que delata resbalón del actuador (normal: 0–2)')
    p.add_argument('--resbalon-ventana', type=float, default=0.5)
    p.add_argument('--escape-g', type=float, default=60.0)
    p.add_argument('--escape-frac', type=float, default=0.35)
    p.add_argument('--tara-espera', type=float, default=10.0)
    p.add_argument('--disparo-plazo', type=float, default=0.040,
                   help='si la fuerza del DOF no cambia en este plazo, el control avanza '
                        'igual: un valor repetido es el valor actual, no informacion vieja')
    p.add_argument('--outdir', default=os.path.join(_HERE, 'data'))
    return p


# ── el controlador ────────────────────────────────────────────────────────
class PI:
    """PI con fuga, banda muerta y cuantización asimétrica.

    Las tres particularidades no son adornos: cada una responde a algo medido.

    **Fuga en el integrador (`lam` < 1), no guarda de posición.** E3.4 midió que la
    fuerza decae 5–10 % en los primeros segundos y **se agota**, y que el actuador
    no cede (0 a −2 counts). Un integrador sin fuga perseguiría indefinidamente una
    caída que ya paró, y acabaría caminando el dedo hacia dentro del objeto.

    **Banda muerta del tamaño del cuanto del dedo.** E3.2 midió que el incremento
    mínimo fiable son **5 unidades cerrando y 3 abriendo**, y que eso vale ~20 g en
    el pulgar pero **60–90 g en el índice**. Pedir al lazo una precisión mejor que
    su cuanto es pedirle que oscile: sin banda muerta, cada corrección se pasa y la
    siguiente corrige de vuelta.

    **Cuantización asimétrica.** El mismo escalón no vale en los dos sentidos: bajo
    carga el dedo devuelve más recorrido del que toma (índice a 1000 g: 2.23
    counts/unidad abriendo contra 1.38 cerrando).

    **Planta dominada por el retardo** (`L/τ ≈ 1`, E3.3): subir `Kp` no acelera el
    lazo, lo hace oscilar. La sintonía empieza con P puro y sube despacio.

    Periodo REFRACTARIO tras cada accion. No estaba en el diseno inicial y la
    primera tanda en hardware lo exigio: el lazo decide cada ~35 ms pero la fuerza
    tarda L + tau ~ 100 ms en responder y ~111 ms en asentar (E3.3). Sin esperar,
    el controlador encadena tres o cuatro escalones ANTES de ver el efecto del
    primero, y con un cuanto de ~80 g por escalon eso son 300 g ya comprometidos:
    medido, pico de 476 g contra una consigna de 250 (+90 %), y el sobreimpulso
    disparo el detector de resbalon.

    No se arregla bajando Kp --el cuanto minimo del dedo pone un suelo a lo que
    cada accion vale-- sino esperando a que la accion se vea. Con la planta
    asentando en 111 ms, ~200 ms de refractario deja ver el resultado completo.
    """

    def __init__(self, kp, ki, lam=0.98, banda=0.0,
                 paso_cierra=5, paso_abre=3, dq_max=20, refractario=0.20):
        self.kp, self.ki, self.lam, self.banda = kp, ki, lam, banda
        self.paso_cierra, self.paso_abre, self.dq_max = paso_cierra, paso_abre, dq_max
        self.refractario = refractario
        self.t_accion = None
        self.resid = 0.0
        self.n_espera = 0
        self.I = 0.0
        self.e = 0.0
        self.n_banda = 0
        self.n_accion = 0

    def reinicia(self):
        self.I = 0.0
        self.resid = 0.0

    def fuerza_pedida(self, e, dt, t=None):
        """Ley de control en FUERZA: devuelve los gramos de correccion pedidos.

        En la pinza la salida no se puede cuantizar aqui: primero hay que pasarla
        por J^-1 para repartirla entre los dos dedos, porque cada comando mueve
        las DOS fuerzas. Cuantizar antes del desacoplo redondearia la correccion
        en el eje equivocado.
        """
        self.e = e
        if (t is not None and self.t_accion is not None
                and t - self.t_accion < self.refractario):
            self.n_espera += 1
            return 0.0
        if abs(e) <= self.banda:
            self.I *= self.lam
            self.n_banda += 1
            return 0.0
        P = self.kp * e
        self.I = self.lam * self.I + self.ki * e * dt
        return P + self.I

    def cuantiza(self, u_cmd, t=None):
        """De correccion continua EN COMANDO a un escalon entero, o cero.

        El cuanto es asimetrico porque el dedo devuelve mas recorrido del que
        toma (E3.2), y por debajo del cuanto el dedo sencillamente no se mueve:
        mandar medio escalon es mandar nada.

        ACUMULA EL RESTO. Descartar lo que no llega al cuanto es letal en la
        pinza: `J^-1` reparte la correccion entre los dos dedos, y si la del
        indice (~3.8 u) cae siempre por debajo de su cuanto (5 u) mientras la
        del pulgar si pasa, se aplica MEDIO par desacoplado. La primera tanda de
        A3 hizo justo eso: el pulgar abrio de 698 a 1000 en 3.6 s, el indice no
        se movio ni una vez, y la bola se cayo. Acumulando, cada dedo acaba
        dando su escalon y la proporcion del par se respeta en promedio.
        """
        self.resid += u_cmd
        cuanto = self.paso_cierra if self.resid > 0 else self.paso_abre
        if abs(self.resid) < cuanto:
            return 0
        dq = int(round(self.resid))
        dq = max(-self.dq_max, min(self.dq_max, dq))
        self.resid -= dq                      # lo que no cupo se guarda
        if dq:
            self.n_accion += 1
            if t is not None:
                self.t_accion = t
        return dq

    def paso(self, ref, medida, dt, t=None):
        """Devuelve (Δq en unidades de comando, e, P, I). Δq>0 CIERRA."""
        e = ref - medida
        self.e = e
        # Refractario: mientras la accion anterior no haya dado su resultado no se
        # decide nada. El integrador tampoco avanza: integrar aqui seria contar el
        # mismo error varias veces mientras la planta aun no ha respondido.
        if (t is not None and self.t_accion is not None
                and t - self.t_accion < self.refractario):
            self.n_espera += 1
            return 0, e, 0.0, self.I
        if abs(e) <= self.banda:
            # Dentro de la banda no se actúa, y el integrador tampoco crece: si
            # siguiera integrando, al salir de la banda saldría con un empujón.
            self.I *= self.lam
            self.n_banda += 1
            return 0, e, 0.0, self.I
        P = self.kp * e
        self.I = self.lam * self.I + self.ki * e * dt
        u = P + self.I
        # cuantización: por debajo del cuanto del sentido, no se manda nada
        cuanto = self.paso_cierra if u > 0 else self.paso_abre
        if abs(u) < cuanto:
            return 0, e, P, self.I
        dq = int(round(u))
        dq = max(-self.dq_max, min(self.dq_max, dq))
        self.n_accion += 1
        if t is not None:
            self.t_accion = t
        return dq, e, P, self.I
