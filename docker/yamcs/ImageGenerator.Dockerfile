FROM maven:3.9.9-eclipse-temurin-17

WORKDIR /workspace

COPY simulator/images/requirements.txt /tmp/image-generator-requirements.txt
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3 python3-pip python3-venv \
    && python3 -m venv /opt/image-generator-venv \
    && /opt/image-generator-venv/bin/pip install --no-cache-dir -r /tmp/image-generator-requirements.txt \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/opt/image-generator-venv/bin:${PATH}"
