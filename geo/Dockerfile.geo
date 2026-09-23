FROM overv/openstreetmap-tile-server:latest

# Updating port of the apache server
# Replace the existing Listen 80 with Listen ${GEO_PORT}
# RUN sed -i "s/Listen 80/Listen 8764/" /etc/apache2/ports.conf


# HealthCheck
# HEALTHCHECK --interval=30s CMD curl --fail http://localhost:${GEO_PORT}/ || exit 1