#!/usr/bin/env bash
# Aisla los entornos conda del entorno global de ROS 2 / robotpkg de esta maquina.
#
# El ~/.bashrc de esta laptop hace `source /opt/ros/humble/setup.bash` y ademas:
#
#     export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
#     export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
#
# Las dos variables van ANTES que lo del entorno conda, asi que se cuelan en
# cualquier env y rompen cosas de dos maneras distintas:
#
#   PYTHONPATH
#     - en "tv" (python 3.10) `import pinocchio` carga la 4.1.0 de robotpkg en
#       vez de la 3.1.0 de conda que fija el README de xr_teleoperate;
#     - en "unitree_sim_env" (python 3.11) ni carga: los .so son de 3.10, y
#       `sim_main.py` empieza con `import pinocchio`.
#
#   LD_LIBRARY_PATH
#     - aunque el modulo Python salga del entorno, el enlazador coge
#       libpinocchio/libboost de /opt/openrobots. Mezclar el binding de una
#       version con las libs de otra da, en el momento de crear el modelo:
#
#           RuntimeError: class version St6vectorIS_ImSaImEESaIS1_EE
#
#       que es Boost.Serialization quejandose de dos registros incompatibles.
#
# La solucion es local a los entornos: hooks de activate.d que limpian las dos
# variables, y de deactivate.d que las devuelven. Ni se toca ~/.bashrc ni se
# rompe ROS fuera de estos entornos.
#
# De LD_LIBRARY_PATH solo se quitan las entradas de /opt/openrobots y /opt/ros;
# el resto (gazebo, drivers) se conserva.
#
# Uso:  ./09_isolate_conda_env.sh [env1 env2 ...]      (por defecto: tv unitree_sim_env)
# Resolucion de conda: busca la instalacion en vez de cablearla.
source "$(dirname "${BASH_SOURCE[0]}")/_conda.sh"
set -euo pipefail

CONDA_BASE="${CONDA_BASE}"
ENVS=("$@")
[ ${#ENVS[@]} -eq 0 ] && ENVS=(tv unitree_sim_env)

for env in "${ENVS[@]}"; do
    PREFIX="$CONDA_BASE/envs/$env"
    if [ ! -d "$PREFIX" ]; then
        echo "  ✗ $env: no existe $PREFIX, se omite"
        continue
    fi

    mkdir -p "$PREFIX/etc/conda/activate.d" "$PREFIX/etc/conda/deactivate.d"

    cat > "$PREFIX/etc/conda/activate.d/00_isolate_from_ros.sh" <<'EOF'
# Aisla este entorno de ROS Humble / robotpkg mientras este activo.
# Ver scripts/09_isolate_conda_env.sh del repo h1_2_utec.
if [ -n "${PYTHONPATH:-}" ]; then
    export _H12_SAVED_PYTHONPATH="$PYTHONPATH"
    unset PYTHONPATH
fi
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    export _H12_SAVED_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
    _h12_clean=""
    _IFS_BAK="$IFS"; IFS=':'
    for _p in $LD_LIBRARY_PATH; do
        case "$_p" in
            /opt/openrobots*|/opt/ros/*) ;;                       # fuera
            "") ;;
            *) _h12_clean="${_h12_clean:+$_h12_clean:}$_p" ;;
        esac
    done
    IFS="$_IFS_BAK"
    if [ -n "$_h12_clean" ]; then export LD_LIBRARY_PATH="$_h12_clean"; else unset LD_LIBRARY_PATH; fi
    unset _h12_clean _p _IFS_BAK
fi
EOF

    cat > "$PREFIX/etc/conda/deactivate.d/00_isolate_from_ros.sh" <<'EOF'
# Devuelve las variables que habia antes de activar el entorno.
if [ -n "${_H12_SAVED_PYTHONPATH:-}" ]; then
    export PYTHONPATH="$_H12_SAVED_PYTHONPATH"
    unset _H12_SAVED_PYTHONPATH
fi
if [ -n "${_H12_SAVED_LD_LIBRARY_PATH:-}" ]; then
    export LD_LIBRARY_PATH="$_H12_SAVED_LD_LIBRARY_PATH"
    unset _H12_SAVED_LD_LIBRARY_PATH
fi
EOF

    # limpiar los hooks de la version anterior, que solo tocaba PYTHONPATH
    rm -f "$PREFIX/etc/conda/activate.d/00_unset_pythonpath.sh" \
          "$PREFIX/etc/conda/deactivate.d/00_restore_pythonpath.sh"

    echo "  ✔ $env: hooks activate.d/deactivate.d instalados"
done

echo
echo "OJO: los hooks solo actuan al hacer 'conda activate'. Si se llama al"
echo "interprete por su ruta absoluta (.../envs/tv/bin/python) las variables"
echo "globales siguen puestas."
