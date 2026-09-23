#!/bin/bash
# generate_ssl_for_ip.sh
# Generates a self-signed SSL certificate for the current NET_IP in .env

set -e

# Get NET_IP from .env
env_file="$(dirname "$0")/../.env"
if [ ! -f "$env_file" ]; then
  echo ".env file not found!"
  exit 1
fi

NET_IP=$(grep '^NET_IP=' "$env_file" | cut -d'=' -f2)
if [ -z "$NET_IP" ]; then
  echo "NET_IP not found in .env!"
  exit 1
fi

CERT_DIR="$(dirname "$0")/certs"
mkdir -p "$CERT_DIR"

openssl req -x509 -nodes -days 365 \
  -newkey rsa:2048 \
  -keyout "$CERT_DIR/nginx.key" \
  -out "$CERT_DIR/nginx.crt" \
  -subj "/CN=$NET_IP" \
  -addext "subjectAltName=IP:$NET_IP"

echo "Certificate generated for IP: $NET_IP in $CERT_DIR"
