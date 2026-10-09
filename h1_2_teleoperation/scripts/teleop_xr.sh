#!/usr/bin/env bash
# Teleoperacion XR con las Quest 3 (brazos + manos Inspire). Atajo a meta/arrancar.sh y meta/parar.sh.
#   ./teleop_xr.sh           arranca (robot COLGADO y en Debug con L2+R2)
#   ./teleop_xr.sh parar
# Detalle: meta/README.md
source "$(dirname "$0")/comun.sh"
case "${1:-}" in
    parar) exec "$R40/meta/parar.sh" ;;
    *) aviso_mueve "brazos y manos siguen al operador; en Debug las piernas NO se sostienen"
       exec "$R40/meta/arrancar.sh" "$@" ;;
esac
