#!/usr/bin/env bash
# Resuelve dónde está conda y el entorno `tv`. Se sourcea desde los demás
# scripts; no hace nada por sí solo.
#
# En la máquina donde nació este repo, conda vivía en /home/utec/miniconda3 y
# esa ruta estaba escrita en nueve scripts. En cualquier otra máquina —otro
# usuario, miniforge en vez de miniconda— ninguno arrancaba. Aquí se busca.
#
# Se puede forzar con CONDA_BASE (la instalación) o CONDA_ENV_TV (el entorno).

if [ -z "${CONDA_BASE:-}" ]; then
    for _b in "$HOME/miniforge3" "$HOME/miniconda3" "$HOME/anaconda3" \
              "$HOME/mambaforge" "$HOME/micromamba" /opt/conda; do
        if [ -d "$_b/envs" ] || [ -f "$_b/etc/profile.d/conda.sh" ]; then
            CONDA_BASE="$_b"; break
        fi
    done
    unset _b
fi
export CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
export CONDA_ENV_TV="${CONDA_ENV_TV:-$CONDA_BASE/envs/tv}"
