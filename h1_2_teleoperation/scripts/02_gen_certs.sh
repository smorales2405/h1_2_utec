#!/usr/bin/env bash
# Genera el par cert.pem / key.pem autofirmado que televuer (Vuer, puerto 8012)
# y teleimager (WebRTC, puerto 60001 en PC2) necesitan para servir HTTPS/WSS.
#
# WebXR exige contexto seguro (HTTPS). El Meta Quest 3 no permite instalar
# CA raiz facilmente, asi que se usa un autofirmado y en el navegador del visor
# se acepta "Advanced -> Proceed to <IP> (unsafe)" una sola vez.
#
# Uso:  ./02_gen_certs.sh [ip_extra_1] [ip_extra_2] ...
# Las IP del host se detectan solas; los argumentos añaden IPs adicionales.
set -euo pipefail

OUT="/home/utec/Documents/h1_2_teleoperation/xr_teleoperate/teleop/televuer"
CONF_DIR="$HOME/.config/xr_teleoperate"

# IPs fijas del despliegue: host por ethernet, PC1/PC2 del robot, manos Inspire
FIXED_IPS=(127.0.0.1 192.168.123.222 192.168.123.161 192.168.123.164 192.168.123.210 192.168.123.211 192.168.11.210)
# IPs vivas de esta laptop (wifi incluida) + las pasadas por argumento
mapfile -t LIVE_IPS < <(ip -4 -brief addr | awk '{for(i=3;i<=NF;i++){split($i,a,"/"); if(a[1]!="") print a[1]}}')
ALL_IPS=($(printf '%s\n' "${FIXED_IPS[@]}" "${LIVE_IPS[@]}" "$@" | sort -u))

CNF=$(mktemp)
{
  echo "[req]"
  echo "distinguished_name = dn"
  echo "x509_extensions = v3_req"
  echo "prompt = no"
  echo "[dn]"
  echo "CN = xr-teleoperate"
  echo "[v3_req]"
  echo "subjectAltName = @alt_names"
  echo "basicConstraints = CA:FALSE"
  echo "keyUsage = digitalSignature, keyEncipherment"
  echo "extendedKeyUsage = serverAuth"
  echo "[alt_names]"
  echo "DNS.1 = localhost"
  i=1
  for ip in "${ALL_IPS[@]}"; do echo "IP.$i = $ip"; i=$((i+1)); done
} > "$CNF"

mkdir -p "$OUT" "$CONF_DIR"
openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
  -keyout "$OUT/key.pem" -out "$OUT/cert.pem" -config "$CNF" -extensions v3_req 2>/dev/null
chmod 600 "$OUT/key.pem"
cp "$OUT/cert.pem" "$OUT/key.pem" "$CONF_DIR/"
rm -f "$CNF"

echo "Certificado generado en:"
echo "  $OUT/{cert.pem,key.pem}"
echo "  $CONF_DIR/{cert.pem,key.pem}   <- ruta que televuer/teleimager buscan por defecto"
echo
openssl x509 -in "$OUT/cert.pem" -noout -text | grep -A3 "Subject Alternative Name"
