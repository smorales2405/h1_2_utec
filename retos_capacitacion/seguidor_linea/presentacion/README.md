# Presentación: seguidor de línea con el Unitree H1-2

Socialización de los retos técnicos – Unitree H1-2 (13 de noviembre de 2026). Beamer 16:9 con el formato de
la presentación de resultados de PI 2 (cajas con posición fija, tablas con cabecera y filas alternas, pies
«Fig.») y los colores y el logo de UTEC (cian `00E1FF`, gris `4E4B4C`, gris claro `E4E4E4`).

| # | Diapositiva | Bloque |
| --- | --- | --- |
| 1–2 | Portada · Equipo y responsabilidades | Apertura |
| 3–4 | Qué pide el reto · Niveles y pistas (PDF frente a montadas) | I · Interpretación del reto |
| 5–6 | Cinco bloques e interfaces (sección 5) · Despliegue en el PC2 | II · Arquitectura |
| 7 | Plan por hitos (sección 8) y línea de tiempo | III · Plan de trabajo |
| 8–13 | Hito 1 (cámara) · Marcha · Hito 2 (percepción) · Estimación y control · Supervisor · Hito 3 | IV · Desarrollo por hitos |
| 14–19 | Registro de las 36 tiradas · Nivel 1 · Nivel 2 · Nivel 3 (la S) · Nivel 3 (hueco y sombra) · Nivel 4 | V · Resultados |
| 20–23 | Resumen (métricas de la sección 9) · Parámetros finales · Conclusiones y mejoras · Gracias | VI · Síntesis |

## Compilar

Con TeX Live (Beamer, fontspec, TikZ, xfp, colortbl, ragged2e) y **XeLaTeX**, dos pasadas:

```bash
./compilar.sh                 # latexmk -xelatex y la comprobación de márgenes
```

En Overleaf: subir la carpeta, `main.tex` como documento principal y XeLaTeX como compilador.

## Vídeos

Cada diapositiva de nivel (15, 16, 18 y 19) tiene un botón que abre `videos/nivelN.mp4` con el reproductor
del sistema (clips de 720p sin audio, hechos con ffmpeg desde `../videos/NivelN_implementacion.MOV`). El PDF
tiene que estar junto a la carpeta `videos/`. Okular, Evince y Acrobat preguntan antes de abrirlo.

```bash
ffmpeg -i ../videos/Nivel1_implementacion.MOV -vf "scale='if(gt(iw,ih),-2,720)':'if(gt(iw,ih),720,-2)',fps=30" \
  -c:v libx264 -crf 27 -pix_fmt yuv420p -an -movflags +faststart videos/nivel1.mp4
```

Tirada de cada vídeo: nivel 2, `104456` (escala 1); nivel 3, `043803` (hueco y sombra en el tramo final);
nivel 4, `053907` (escala 1).

## Archivos

| Archivo o carpeta | Contenido |
| --- | --- |
| `main.tex` | Documento principal y orden de las diapositivas. |
| `estilo.tex` | Página (960 × 540 bp), fuentes y colores UTEC. |
| `estilo_seguidor.tex` | Cabecera, pie, cajas de texto, tablas, cifras, filas problema/cambio/resultado, botón de vídeo y estilos de los diagramas. |
| `diapositivas/` | Una diapositiva por archivo. |
| `graficas/generar_graficas.py` | Gráficas (PDF al tamaño de su hueco) a partir de `../datos/` y del simulador. |
| `graficas/recortar_fotos.py` | Fotos: fotogramas de los vídeos y de la cámara IR, recortados a su hueco. |
| `imagenes/` | Logos de UTEC, fotos y figuras de `../docs/img/`. |
| `verificar_margenes.py` | Comprueba sobre el PDF que nada se salga, pise el logo o el pie, ni se solape. |

Las cifras salen de `../docs/RESULTADOS.md`, de `herramientas/metricas.py` y de los registros de
`../datos/`. Para rehacer gráficas y fotos (desde esta carpeta, con los datos y los vídeos en su sitio):

```bash
python3 graficas/generar_graficas.py
python3 graficas/recortar_fotos.py
```

Coordenadas en bp desde la esquina superior izquierda; en `\Texto` y `\Parrafo`, la `y` es la línea base
de la primera línea, y en `\Imagen`, el borde superior.
