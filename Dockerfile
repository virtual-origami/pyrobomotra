FROM python:3.8.3-slim-buster AS base

# Dedicated Workdir for App
WORKDIR /pyrobomotra

# Do not run as root
RUN useradd -m -r pyrobomotra && \
    chown pyrobomotra /pyrobomotra

COPY requirements.txt /pyrobomotra
RUN pip3 install -r requirements.txt

FROM base AS src
COPY . /pyrobomotra
# install pyrobomotra here as a python package
RUN pip3 install .

USER pyrobomotra

COPY scripts/docker-entrypoint.sh /entrypoint.sh

# Use the `robot-motion-tracker` binary as Application
FROM src AS prod
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["robot-motion-tracker", "-c", "config.yaml"]