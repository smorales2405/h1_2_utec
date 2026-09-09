#!/usr/bin/env bash
# Encuentra la IP del Meta Quest 3 en la red del router.
#
#   ./scripts/busca_quest.sh
#
# El visor no enseña su propia IP por ningún menú accesible, así que hay que
# escanear. Se mira cada interfaz con IPv4 privada que NO sea la del robot
# (192.168.123.x, que es una red punto a punto sin visor), y se descartan la
# propia laptop y la puerta de enlace. Lo que queda es el visor.
#
# Necesita `arp-scan` y sudo: ARP no depende de ICMP, y muchos routers y el
# propio visor no responden a ping.
set -euo pipefail

command -v arp-scan >/dev/null || {
    echo "  falta arp-scan:  sudo apt install arp-scan"; exit 1; }

encontrados=0
while read -r iface cidr; do
    ip="${cidr%%/*}"
    gw=$(ip route show dev "$iface" default 2>/dev/null | awk '{print $3; exit}')
    echo "══ $iface ($ip) ══"
    salida=$(sudo arp-scan --interface="$iface" --localnet 2>/dev/null \
             | grep -E "^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+" || true)
    if [ -z "$salida" ]; then
        echo "   nadie responde en este segmento"
        continue
    fi
    while read -r vip vmac resto; do
        etiqueta=""
        [ "$vip" = "$ip" ] && etiqueta="  <- esta laptop"
        [ -n "$gw" ] && [ "$vip" = "$gw" ] && etiqueta="  <- router (puerta de enlace)"
        if [ -z "$etiqueta" ]; then
            etiqueta="  <-- CANDIDATO A VISOR"
            encontrados=$((encontrados + 1))
            quest_ip="$vip"; quest_host="$ip"
        fi
        printf "   %-16s %-18s %s%s\n" "$vip" "$vmac" "$resto" "$etiqueta"
    done <<< "$salida"
done < <(ip -br -4 addr | awk '$1!="lo" && $3 ~ /^(10\.|172\.1[6-9]\.|172\.2[0-9]\.|172\.3[01]\.|192\.168\.)/ \
                               && $3 !~ /^192\.168\.123\./ {print $1, $3}')

echo
if [ "$encontrados" -eq 1 ]; then
    echo "  Visor:  $quest_ip"
    echo "  En el navegador del Quest, abre:"
    echo "      https://${quest_host}:8012"
    echo
    if openssl x509 -in "$HOME/.config/xr_teleoperate/cert.pem" -noout -ext subjectAltName 2>/dev/null \
       | grep -q "IP Address:${quest_host}\b"; then
        echo "  ✔ el certificado cubre ${quest_host}"
    else
        echo "  ⚠ el certificado NO cubre ${quest_host}. Regenéralo:"
        echo "      ./scripts/02_gen_certs.sh"
        echo "    y vuelve a aceptar el aviso en el visor."
    fi
elif [ "$encontrados" -eq 0 ]; then
    echo "  No hay ningún dispositivo aparte de la laptop y el router."
    echo "  ¿Está el visor encendido y con el cable puesto? Se duerme al quitárselo."
else
    echo "  Hay $encontrados candidatos. El visor es el que no reconozcas;"
    echo "  para distinguirlo, desconéctalo y vuelve a escanear."
fi
