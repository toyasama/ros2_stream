#!/bin/bash

RTSP_URL="rtsp://localhost:8554/livestream"
MAX_WAIT=20
WAIT_INTERVAL=1
TIME_WAITED=0

echo "Waiting for RTSP stream to be available at: ${RTSP_URL}"

while true; do
  if ffprobe -v error -rtsp_transport tcp -i "$RTSP_URL" -show_streams > /dev/null 2>&1; then
    echo "RTSP stream is available. Starting viewer..."
    exec "$@"
  else
    echo "Waiting for RTSP stream at $RTSP_URL..."
  fi

  sleep $WAIT_INTERVAL
  TIME_WAITED=$((TIME_WAITED + WAIT_INTERVAL))

  if [ $TIME_WAITED -ge $MAX_WAIT ]; then
    echo "Timeout: RTSP stream not available after ${MAX_WAIT}s."
    exit 1
  fi
done

echo "Done"
