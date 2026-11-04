#!/usr/bin/env bash
# Teleoperacion XR con las Quest 3 (brazos + manos Inspire). Atajo a arrancar.sh y parar.sh.
#   ./teleop_xr.sh           arranca (robot COLGADO y en Debug con L2+R2)
#   ./teleop_xr.sh parar
# Detalle: ~/utec/teleoperacion/README.md en el robot
source "$(dirname "$0")/comun.sh"
case "${1:-}" in
    parar) exec "$TELEOP/parar.sh" ;;
    *) aviso_mueve "brazos y manos siguen al operador; en Debug las piernas NO se sostienen"
       exec "$TELEOP/arrancar.sh" "$@" ;;
esac
