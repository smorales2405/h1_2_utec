"""Prueba: encontrar un objeto circular azul y decir a qué distancia está.

    ros2 run h1_2_vision detector_azul --ros-args -p diametro:=0.065

Es la prueba de que la cámara sirve para algo más que mirarla: coge los
fotogramas de `head_camera`, busca la mancha azul más redonda y publica dónde
está en el espacio, además de decirlo por consola.

**La distancia se estima por tamaño aparente, no se mide.** Por el `videohub`
no llega profundidad —ni la llegará mientras la D435i cuelgue del PC1—, así que
lo único que queda es el modelo pinhole de toda la vida:

        Z = fx · diámetro_real / diámetro_en_píxeles

Lo que eso implica, y conviene tener presente antes de fiarse de un número:

  * **Hay que decirle el diámetro real** del objeto con `-p diametro:=` (en
    metros). El resultado escala LINEALMENTE con ese dato: si te equivocas un
    10% en el diámetro, te equivocas un 10% en la distancia.
  * Los intrínsecos que usa son los que publica `head_camera`, que salen del
    campo de visión de catálogo, **no de una calibración**. Cuentan como otra
    fuente de error de un pequeño porcentaje.
  * El radio en píxeles de una mancha comprimida en JPEG baila un píxel o dos
    de un fotograma al siguiente, y a 3 m un píxel ya son varios centímetros.

El radio que va a la fórmula **sale del área** (`r = √(área/π)`), no del círculo
que encierra el contorno. Medido sobre círculos sintéticos con ruido y
compresión JPEG, `minEnclosingCircle` sesga +4% de media y hasta +8% en manchas
pequeñas —encierra los centros de los píxeles del borde, así que siempre sobra
casi un píxel—, mientras que el área se queda en +1,5% de media y +2,8% en el
peor caso. El círculo que encierra se sigue usando para pintar y para medir la
redondez, que es lo suyo.

Con todo eso, lo razonable es leerlo como «está a un metro y pico», no como una
medida. Para medir de verdad hacen falta profundidad o una calibración.

**Se puede contrastar sin nada más que una cinta métrica**: mides la distancia
real, la pasas con `-p distancia_real:=1.20`, y cada línea dice cuánto se
equivoca. Al salir da el factor de corrección, que absorbe de golpe el error
del diámetro supuesto y el de los intrínsecos sin calibrar.

Publica, además de imprimir:

    /objeto_azul/imagen/compressed   la imagen con el círculo pintado (para RViz)
    /objeto_azul/punto               geometry_msgs/PointStamped en el marco óptico
    /objeto_azul/marcador            visualization_msgs/Marker, una esfera del
                                     tamaño real puesta a la distancia estimada
"""
from __future__ import annotations

import array
import math
import time
from typing import NamedTuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from visualization_msgs.msg import Marker

PARAMS = {
    "diametro": 0.065,      # diámetro REAL del objeto, en metros
    "topic": "/head_camera/image_raw/compressed",
    "topic_info": "/head_camera/camera_info",
    # Azul en HSV de OpenCV: el tono va de 0 a 179, y el azul cae sobre 100-130.
    "h_min": 95,
    "h_max": 130,
    "s_min": 110,           # saturación: por debajo es gris azulado, como el suelo
    "v_min": 60,            # valor: por debajo es negro, y el tono no significa nada
    "radio_min_px": 12,     # por debajo de esto es ruido
    "circularidad_min": 0.70,   # área real / área del círculo que lo encierra
    "anotar": True,
    "marcador": True,
    "ventana": False,       # ventana de OpenCV, si hay pantalla
    "best_effort": False,
    "report_every": 1.0,    # segundos entre líneas por consola; 0 las silencia
    # Si mides la distancia de verdad con una cinta y la pones aquí, cada línea
    # dice cuánto se equivoca, y al salir da el factor con el que corregirlo.
    # Es lo más parecido a una calibración que se puede hacer sin profundidad.
    "distancia_real": 0.0,
}


class Hallazgo(NamedTuple):
    centro: tuple          # (u, v) en píxeles
    radio: float           # √(área/π): el que va a la distancia
    radio_borde: float     # minEnclosingCircle: para pintar y para la redondez
    circularidad: float


class DetectorAzul(Node):

    def __init__(self):
        super().__init__("h1_2_detector_azul")
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)

        qos = qos_profile_sensor_data if self.p("best_effort") else 10
        self.create_subscription(CompressedImage, self.p("topic"), self._on_image, qos)
        self.create_subscription(CameraInfo, self.p("topic_info"), self._on_info, qos)

        self.pub_img = (self.create_publisher(
            CompressedImage, "/objeto_azul/imagen/compressed", qos)
            if self.p("anotar") else None)
        self.pub_pto = self.create_publisher(PointStamped, "/objeto_azul/punto", qos)
        self.pub_mrk = (self.create_publisher(Marker, "/objeto_azul/marcador", qos)
                        if self.p("marcador") else None)

        self.fx = self.fy = self.cx = self.cy = None
        self._ultimo_aviso = 0.0
        self._fotogramas = 0
        self._aciertos = 0
        self._distancias: list[float] = []

        self.get_logger().info(
            f"buscando un objeto azul de {self.p('diametro') * 100:.1f} cm de diámetro "
            f"en {self.p('topic')}")
        self.get_logger().warn(
            "la distancia se ESTIMA por tamaño aparente; escala con el diámetro "
            "que se le diga y con unos intrínsecos sin calibrar")

    def p(self, nombre):
        return self.get_parameter(nombre).value

    def _on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    # -- detección --------------------------------------------------------
    def _busca_azul(self, bgr) -> "Hallazgo | None":
        """El objeto azul más grande del fotograma, o None si no hay ninguno.

        Color y forma, las dos cosas. Solo con el color, cualquier reflejo del
        suelo —que aquí es gris azulado— entra; la circularidad es lo que
        distingue una pelota de una sombra con la misma tonalidad.
        """
        # El JPEG deja bloques de 8x8 en las zonas planas; el desenfoque los
        # borra antes de umbralizar, que si no la máscara sale con agujeros.
        hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mascara = cv2.inRange(
            hsv,
            np.array([self.p("h_min"), self.p("s_min"), self.p("v_min")], np.uint8),
            np.array([self.p("h_max"), 255, 255], np.uint8))
        nucleo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, nucleo)

        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        mejor = None
        for c in contornos:
            (x, y), r_borde = cv2.minEnclosingCircle(c)
            if r_borde < self.p("radio_min_px"):
                continue
            area = cv2.contourArea(c)
            area_circulo = math.pi * r_borde * r_borde
            circularidad = area / area_circulo if area_circulo else 0.0
            if circularidad < self.p("circularidad_min"):
                continue
            if mejor is None or r_borde > mejor.radio_borde:
                mejor = Hallazgo((x, y), math.sqrt(area / math.pi), r_borde,
                                 circularidad)
        return mejor

    def _distancia(self, radio_px: float) -> float:
        """Z = fx · D / d, con d el diámetro en píxeles."""
        fx = self.fx if self.fx else 1386.4      # el nominal, si aún no hay CameraInfo
        return fx * float(self.p("diametro")) / (2.0 * radio_px)

    # -- bucle ------------------------------------------------------------
    def _on_image(self, msg: CompressedImage):
        bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        self._fotogramas += 1

        hallazgo = self._busca_azul(bgr)
        if hallazgo is not None:
            u, v = hallazgo.centro
            z = self._distancia(hallazgo.radio)
            fx = self.fx or 1386.4
            fy = self.fy or 1388.6
            cx = self.cx if self.cx is not None else bgr.shape[1] / 2.0
            cy = self.cy if self.cy is not None else bgr.shape[0] / 2.0
            # Marco óptico: x a la derecha, y hacia abajo, z hacia adelante.
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            self._aciertos += 1
            self._distancias.append(z)
            self._publica(msg.header, x, y, z)
            if self._toca_trazar():
                self._traza(z, hallazgo, (u, v), x, y)
        else:
            self._traza_vacio()

        if self.pub_img is not None:
            self._publica_imagen(msg.header, bgr, hallazgo)
        if self.p("ventana"):
            cv2.imshow("detector_azul", cv2.resize(bgr, (960, 540)))
            cv2.waitKey(1)

    def _publica(self, cabecera, x, y, z):
        pto = PointStamped()
        pto.header = cabecera
        pto.point.x, pto.point.y, pto.point.z = x, y, z
        self.pub_pto.publish(pto)

        if self.pub_mrk is None:
            return
        d = float(self.p("diametro"))
        m = Marker()
        m.header = cabecera
        m.ns, m.id = "objeto_azul", 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = d
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.4, 1.0, 0.85
        # Si el objeto desaparece, la esfera se va sola en medio segundo.
        m.lifetime.sec = 0
        m.lifetime.nanosec = 500_000_000
        self.pub_mrk.publish(m)

    def _publica_imagen(self, cabecera, bgr, hallazgo):
        if hallazgo is not None:
            z = self._distancia(hallazgo.radio)
            centro = (int(hallazgo.centro[0]), int(hallazgo.centro[1]))
            radio = hallazgo.radio_borde
            cv2.circle(bgr, centro, int(radio), (0, 255, 255), 4)
            cv2.circle(bgr, centro, 5, (0, 0, 255), -1)
            cv2.putText(bgr, f"{z:.2f} m", (centro[0] - 60, centro[1] - int(radio) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 255, 255), 4)
            cv2.putText(bgr, f"r={radio:.0f}px", (centro[0] - 60, centro[1] + int(radio) + 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
        else:
            cv2.putText(bgr, "sin objeto azul", (40, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)

        ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        out = CompressedImage()
        out.header = cabecera
        out.format = "jpeg"
        out.data = array.array("B", jpeg.tobytes())   # nunca `bytes`: ver el README
        self.pub_img.publish(out)

    # -- consola ----------------------------------------------------------
    def _toca_trazar(self) -> bool:
        cada = float(self.p("report_every"))
        if cada <= 0:
            return False
        ahora = time.monotonic()
        if ahora - self._ultimo_aviso < cada:
            return False
        self._ultimo_aviso = ahora
        return True

    def _traza(self, z, hallazgo, centro, x, y):
        real = float(self.p("distancia_real"))
        contraste = ("" if real <= 0 else
                     f" · real {real:.2f} m · error {(z - real) / real * 100:+.1f}%")
        print(f"  ● azul a {z:5.2f} m  ({z * 100:5.1f} cm)   "
              f"radio {hallazgo.radio:5.1f} px · redondez {hallazgo.circularidad:.2f} · "
              f"centro ({centro[0]:4.0f},{centro[1]:4.0f}) · "
              f"xyz ({x:+.2f}, {y:+.2f}, {z:.2f}) m{contraste}", flush=True)

    def _traza_vacio(self):
        if not self._toca_trazar():
            return
        print("  · sin objeto azul en el encuadre", flush=True)

    def resumen(self):
        if not self._distancias:
            return (f"\n  {self._fotogramas} fotogramas, ninguno con objeto azul.\n"
                    f"  Si lo había: prueba a bajar `s_min` o `circularidad_min`, "
                    f"o mira la máscara con `-p ventana:=true`.")
        d = np.array(self._distancias)
        texto = (f"\n  {self._aciertos}/{self._fotogramas} fotogramas con objeto "
                 f"({100 * self._aciertos / max(1, self._fotogramas):.0f}%)\n"
                 f"  distancia: media {d.mean():.2f} m · mediana {np.median(d):.2f} m · "
                 f"min {d.min():.2f} · máx {d.max():.2f} · "
                 f"desviación {d.std() * 100:.1f} cm")
        real = float(self.p("distancia_real"))
        if real > 0:
            medida = float(np.median(d))
            factor = real / medida
            texto += (
                f"\n  contra la cinta: real {real:.2f} m, medido {medida:.2f} m, "
                f"error {(medida - real) / real * 100:+.1f}%"
                f"\n  para corregirlo: -p diametro:="
                f"{float(self.p('diametro')) * factor:.4f}  "
                f"(el factor {factor:.3f} absorbe el diámetro y los intrínsecos)")
        return texto


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = DetectorAzul()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            print(nodo.resumen())
            cv2.destroyAllWindows()
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
