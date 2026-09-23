FROM python:3.9

# Install required dependencies
RUN apt-get update && apt-get install -y \
    netcat-traditional \
    libgl1-mesa-dri \
    libgl1 \
    libglu1-mesa && \
    rm -rf /var/lib/apt/lists/*

# Copy the python dependencies
COPY alg/requirements.txt /app/requirements.txt
# Install Python dependencies
RUN pip install -r /app/requirements.txt

# copy the entrypoint bash script
COPY alg/entrypoint.alg.sh .

# Copy the python application
COPY alg/app /app

# HealthCheck
HEALTHCHECK --interval=30s CMD curl --fail http://localhost:${ALG_API_PORT}/alg/healthCheck || exit 1

# run the entrypoint bash script
ENTRYPOINT ["/bin/bash", "./entrypoint.alg.sh"]