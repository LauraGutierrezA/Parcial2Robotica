#!/bin/bash
# Corre el ciclo completo de pick-and-place: 4A -> 4B -> 4C -> 4D
# Requiere que demo.launch.py ya este corriendo (RViz abierto, robot en home).

set -e  # si algun paso falla, se detiene aqui (no sigue con el siguiente)

WS=~/ws_kuka_R6_R700-2

echo "=========================================="
echo "  PARTE 4A: home -> pre-pick"
echo "=========================================="
python3 $WS/demo_4a.py

echo ""
echo "=========================================="
echo "  PARTE 4B: pre-pick -> pick (agarra pieza)"
echo "=========================================="
python3 $WS/4b_v2.py

echo ""
echo "=========================================="
echo "  PARTE 4C: pick -> pre-place"
echo "=========================================="
python3 $WS/traslado_4c.py

echo ""
echo "=========================================="
echo "  PARTE 4D: pre-place -> place (deposita pieza)"
echo "=========================================="
python3 $WS/4d_simple.py

echo ""
echo "=========================================="
echo "  CICLO COMPLETO TERMINADO"
echo "=========================================="
