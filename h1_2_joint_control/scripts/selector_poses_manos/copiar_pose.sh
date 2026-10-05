#!/usr/bin/env bash
# Copia UNA rutina de poses/ de este PC a la carpeta de rutinas del selector en el robot, por su número:
#   ./copiar_pose.sh 3          copia poses/3_*.json  ->  ~/utec/selector_poses_manos/poses/ en el robot
# Si en el robot ya hay rutinas con ese número (3_*.json, aunque se llamen distinto), las sustituye.
# Sirve desde cualquier PC y cualquier usuario: busca poses/ junto a este script, no en una ruta fija.
#
# Variables, por si hacen falta:
#   ROBOT_SSH=unitree@192.168.123.164 ./copiar_pose.sh 3     por cable (por defecto, la WiFi: unitree@192.168.0.143)
#   DESTINO=otra/carpeta ./copiar_pose.sh 3                   carpeta en el robot, relativa a su $HOME
#
# Usa una sola conexión SSH: sin clave autorizada, pide la contraseña del robot una vez. La rutina se escribe
# primero con un nombre temporal y luego se renombra, así que el selector, aunque esté en marcha, nunca lee un
# archivo a medias: la versión nueva se usa la próxima vez que se elija ese número.
set -euo pipefail

ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
DESTINO=${DESTINO:-utec/selector_poses_manos/poses}
POSES="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/poses"
case "$DESTINO" in /*) DONDE=$DESTINO ;; *) DONDE="~/$DESTINO" ;; esac   # solo para los mensajes

uso() { sed -n '2,4p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -eq 1 ] || uso 1
case "$1" in -h|--help) uso 0 ;; esac
[[ "$1" =~ ^[0-9]+$ ]] || { echo "El argumento es el número de la rutina (por ejemplo 3), no '$1'." >&2; exit 1; }
N=$((10#$1))                                    # "03" es la rutina 3

# La rutina en este PC: exactamente una con ese número.
shopt -s nullglob
locales=("$POSES/${N}"_*.json)
if [ ${#locales[@]} -eq 0 ]; then
    echo "No hay ninguna rutina ${N}_*.json en $POSES" >&2
    exit 1
fi
if [ ${#locales[@]} -gt 1 ]; then
    echo "Hay ${#locales[@]} rutinas con el número $N en $POSES; deja solo una:" >&2
    printf '  %s\n' "${locales[@]##*/}" >&2
    exit 1
fi
ARCHIVO=${locales[0]}
NOMBRE=${ARCHIVO##*/}

# Que sea un JSON válido del H1-2 antes de mandarlo (si hay python3 en este PC).
if command -v python3 >/dev/null; then
    python3 - "$ARCHIVO" <<'EOF' || exit 1
import json, sys
ruta = sys.argv[1]
try:
    rutina = json.load(open(ruta, encoding="utf-8"))
except Exception as e:
    sys.exit(f"{ruta} no es un JSON válido: {e}")
if rutina.get("robot") not in (None, "unitree_h1_2"):
    sys.exit(f"{ruta} es para '{rutina.get('robot')}', no para el H1-2")
if not rutina.get("pasos"):
    sys.exit(f"{ruta} no tiene pasos")
EOF
fi
SUMA=$(sha256sum "$ARCHIVO" | cut -d' ' -f1)
[ "$N" -eq 0 ] && echo "Ojo: la rutina 0 es la pose segura, la que usa el selector al salir."

# En el robot, con el archivo por la entrada estándar: comprobar la carpeta, escribir en un temporal,
# borrar las otras rutinas con ese número y renombrar. Todo en sh POSIX y en una sola conexión.
echo "Copiando $NOMBRE a $ROBOT_SSH:$DONDE/ ..."
RESULTADO=$(ssh -o ConnectTimeout=10 "$ROBOT_SSH" "$(printf 'D=%q W=%q N=%q F=%q S=%q\n' "$DESTINO" "$DONDE" "$N" "$NOMBRE" "$SUMA")"'
cd "$D" 2>/dev/null || { echo "ERROR: no existe $W en el robot (copia antes la carpeta del selector)"; exit 1; }
t=$(mktemp ".copiar_pose.XXXXXX") || exit 1
cat > "$t" || { rm -f "$t"; exit 1; }
if [ "$(sha256sum "$t" | cut -d" " -f1)" != "$S" ]; then rm -f "$t"; echo "ERROR: la copia llegó corrupta"; exit 1; fi
chmod 664 "$t"
for v in "${N}"_*.json; do
    [ -e "$v" ] || continue
    if [ "$v" = "$F" ]; then echo "SUSTITUIDA: $v"; else rm -f -- "$v" && echo "BORRADA: $v"; fi
done
mv -f -- "$t" "$F" && echo "OK"
' < "$ARCHIVO") || true

if [ -n "$RESULTADO" ]; then
    printf '%s\n' "$RESULTADO" | sed -n '/^OK$/!s/^/  /p'
fi
if [ "$(echo "$RESULTADO" | tail -n 1)" != "OK" ]; then
    echo "No se copió." >&2
    exit 1
fi
echo "Hecho: $NOMBRE en el robot (sha256 comprobado). En el selector sale con el número $N."
