#!/usr/bin/env bash
touch /tmp/z1pro_video_loop.run
while [ -f /tmp/z1pro_video_loop.run ]; do
  /home/unitree/go2_fastlio_ws/scripts/z1pro_capture.sh record 20 || true
  sleep 1
done
