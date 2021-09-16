#!/bin/bash
set -e

if [ "${1:0:1}" = '-' ]; then
    set -- robot-motion-tracker "$@"
fi

exec "$@"
