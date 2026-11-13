#!/usr/bin/env bash
# Compila la presentación con XeLaTeX (dos pasadas: las posiciones son absolutas) y comprueba los márgenes.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if command -v latexmk >/dev/null 2>&1; then
  latexmk -xelatex -interaction=nonstopmode -halt-on-error main.tex
else
  xelatex -interaction=nonstopmode -halt-on-error main.tex
  xelatex -interaction=nonstopmode -halt-on-error main.tex
fi
python3 verificar_margenes.py main
