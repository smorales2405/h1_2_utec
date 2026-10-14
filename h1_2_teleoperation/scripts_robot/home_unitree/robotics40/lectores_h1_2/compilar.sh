#!/usr/bin/env bash
# Recompila las herramientas C++ contra el unitree_sdk2 instalado en /opt/unitree_robotics.
# Rehace build/ desde cero: el antiguo se configuro en ~/lectores_h1_2 (la ruta de antes de moverlo).
source "$(dirname "$0")/../comun.sh"   # fuera del robot, se reenvia al robot
set -e
cd "$(dirname "$0")"
rm -rf build
cmake -B build -Dunitree_sdk2_DIR=/opt/unitree_robotics/lib/cmake/unitree_sdk2   # como la primera vez
cmake --build build -j4
ls build | grep -v -e CMake -e Makefile -e cmake
