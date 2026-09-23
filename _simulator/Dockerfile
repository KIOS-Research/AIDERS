FROM python:3.10-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    wget \
    xz-utils \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
        
# Install FFmpeg 7.x static build — required for NVENC SDK 12.x (driver >= 520)
# Ubuntu 20.04 ships FFmpeg 4.2 which uses the old preset GUID API, removed in SDK 12.
RUN wget -q -O /tmp/ffmpeg-N-122607-g50bcc96a75-linux64-gpl.tar.xz \
        "https://github.com/BtbN/FFmpeg-Builds/releases/download/autobuild-2026-01-31-12-57/ffmpeg-N-122607-g50bcc96a75-linux64-gpl.tar.xz" && \
    tar -xf /tmp/ffmpeg-N-122607-g50bcc96a75-linux64-gpl.tar.xz -C /tmp && \
    ffmpeg_dir="$(find /tmp -maxdepth 1 -type d -name 'ffmpeg-*linux64-gpl' | head -n 1)" && \
    test -n "$ffmpeg_dir" && test -x "$ffmpeg_dir/bin/ffmpeg" && test -x "$ffmpeg_dir/bin/ffprobe" && \
    mv "$ffmpeg_dir/bin/ffmpeg" /usr/local/bin/ffmpeg && \
    mv "$ffmpeg_dir/bin/ffprobe" /usr/local/bin/ffprobe && \
    rm -rf /tmp/ffmpeg*

# Copy the python dependencies
COPY requirements.txt /app/requirements.txt
# Install Python dependencies
RUN pip install -r /app/requirements.txt

# copy the entrypoint bash script
COPY entrypoint.sh .

# Copy the python application
COPY app /app

# run the entrypoint bash script
ENTRYPOINT ["/bin/bash", "./entrypoint.sh"]

