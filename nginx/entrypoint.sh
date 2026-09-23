#!/bin/bash
set -e

# Generate SSL config
# envsubst '
#     $NGINX_PORT $WS_IP $WS_PORT $WSI_IP $WSI_PORT $WSM_IP $WSM_PORT
#     ' < /etc/nginx/conf.d/ssl.conf.template > /etc/nginx/conf.d/ssl.conf # SSLEnabled

# Generate default config.
# The whitelist is derived from the template itself: only ${VAR} references are
# substituted, so nginx's own $variables (e.g. $http_host) are left untouched.
TEMPLATE=/etc/nginx/conf.d/default.conf.template
VARS=$(grep -oE '\$\{[A-Za-z_][A-Za-z0-9_]*\}' "$TEMPLATE" | sort -u | tr -d '${}' | sed 's/^/$/' | tr '\n' ' ')

for var in $(echo "$VARS" | tr -d '$'); do
    if [ -z "${!var:-}" ]; then
        echo "WARNING: $var is referenced by $TEMPLATE but is not set in the environment" >&2
    fi
done

envsubst "$VARS" < "$TEMPLATE" > /etc/nginx/conf.d/default.conf

echo "Nginx configuration files generated."

# Start nginx
exec nginx -g 'daemon off;'
