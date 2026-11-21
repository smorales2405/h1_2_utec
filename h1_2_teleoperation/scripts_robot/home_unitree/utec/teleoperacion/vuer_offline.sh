#!/usr/bin/env bash
# Hace que la pagina de Vuer no necesite Internet al entrar en VR.
#
# Al pulsar "Virtual Reality", la libreria WebXR de la pagina (dentro de vuer/client_build) descarga los modelos de
# las manos y de los mandos de
#     https://cdn.jsdelivr.net/npm/@webxr-input-profiles/assets@1.0/dist/profiles/
# Los descarga el navegador de las Quest, no el robot. Sin Internet en la WiFi, la escena se congela al entrar en VR
# (comprobado el 2026-11-21). Este script:
#   1. descarga esos modelos (hace falta Internet, solo esta vez) en la carpeta que ya sirve Vuer:
#      client_build/assets/webxr-input-profiles/  ->  https://<robot>:8012/assets/webxr-input-profiles/
#   2. cambia esa direccion en la pagina por la del propio robot: location.origin + "/assets/webxr-input-profiles/".
# Modifica el paquete vuer instalado en env_tv: si se reinstala, hay que volver a pasarlo.
#
#   ./vuer_offline.sh              descarga lo que falte y parchea la pagina (se puede repetir)
#   ./vuer_offline.sh comprobar    dice si esta aplicado, sin cambiar nada
#   ./vuer_offline.sh deshacer     deja la pagina como venia (los modelos descargados se quedan)
#
# Despues, en las Quest, borrar una vez los datos de navegacion: la pagina vieja puede seguir en su cache varios dias.
set -euo pipefail
AQUI="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1
PY=$AQUI/env_tv/bin/python
CB=$("$PY" -c 'import os, vuer; print(os.path.join(os.path.dirname(vuer.__file__), "client_build"))')

"$PY" - "$CB" "${1:-aplicar}" <<'PY'
import base64, hashlib, json, os, sys, urllib.request

CB, MODO = sys.argv[1], sys.argv[2]
VERSION = "1.0.20"   # la que da hoy jsDelivr para @1.0, la que pedia la pagina
CDN = "https://cdn.jsdelivr.net/npm/@webxr-input-profiles/assets@1.0/dist/profiles/"
LOCAL = 'location.origin+"/assets/webxr-input-profiles/"'
DEST = os.path.join(CB, "assets", "webxr-input-profiles")


def js_de_la_pagina():
    for raiz, _, nombres in os.walk(os.path.join(CB, "assets")):
        for n in nombres:
            if n.endswith(".js"):
                yield os.path.join(raiz, n)


def estado():
    con_cdn = [f for f in js_de_la_pagina() if CDN in open(f, encoding="utf-8").read()]
    con_local = [f for f in js_de_la_pagina() if LOCAL in open(f, encoding="utf-8").read()]
    lista = os.path.isfile(os.path.join(DEST, "profilesList.json"))
    manos = all(os.path.isfile(os.path.join(DEST, "generic-hand", n)) for n in ("profile.json", "left.glb", "right.glb"))
    return con_cdn, con_local, lista and manos


def descargar():
    api = f"https://data.jsdelivr.com/v1/packages/npm/@webxr-input-profiles/assets@{VERSION}?structure=flat"
    archivos = [f for f in json.load(urllib.request.urlopen(api, timeout=30))["files"]
                if f["name"].startswith("/dist/profiles/")]
    nuevos = 0
    for f in archivos:
        rel = f["name"][len("/dist/profiles/"):]
        ruta = os.path.join(DEST, rel)
        if os.path.isfile(ruta) and os.path.getsize(ruta) == f["size"]:
            continue
        datos = urllib.request.urlopen(
            f"https://cdn.jsdelivr.net/npm/@webxr-input-profiles/assets@{VERSION}{f['name']}", timeout=60).read()
        if len(datos) != f["size"] or base64.b64encode(hashlib.sha256(datos).digest()).decode() != f["hash"]:
            sys.exit(f"ERROR: {rel} no coincide con el tamano o el sha256 que da jsDelivr")
        os.makedirs(os.path.dirname(ruta), exist_ok=True)
        with open(ruta + ".parte", "wb") as g:
            g.write(datos)
        os.replace(ruta + ".parte", ruta)
        nuevos += 1
    print(f"modelos: {len(archivos)} archivos en {DEST} ({nuevos} descargados ahora)")


def parchear():
    n = 0
    for f in js_de_la_pagina():
        texto = open(f, encoding="utf-8").read()
        if CDN not in texto:
            continue
        if not os.path.exists(f + ".orig"):
            with open(f + ".orig", "w", encoding="utf-8") as g:
                g.write(texto)
        nuevo = texto.replace(f'"{CDN}"', LOCAL)
        if CDN in nuevo:
            sys.exit(f"ERROR: {f} usa la direccion de otra forma; no se toca")
        with open(f, "w", encoding="utf-8") as g:
            g.write(nuevo)
        n += 1
    print(f"pagina: {n} archivos parcheados")


def deshacer():
    n = 0
    for f in js_de_la_pagina():
        if os.path.exists(f + ".orig"):
            os.replace(f + ".orig", f)
            n += 1
    print(f"pagina: {n} archivos devueltos al original")


if MODO == "aplicar":
    descargar()
    parchear()
elif MODO == "deshacer":
    deshacer()
elif MODO != "comprobar":
    sys.exit("uso: vuer_offline.sh [comprobar|deshacer]")
con_cdn, con_local, modelos = estado()
print(f"estado: {len(con_cdn)} archivos piden los modelos a Internet, {len(con_local)} al robot; "
      f"modelos locales {'si' if modelos else 'NO'}")
print("OFFLINE: si" if not con_cdn and con_local and modelos else "OFFLINE: no")
PY
