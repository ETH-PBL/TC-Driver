#!/bin/bash

# Source this file from /opt/riders/setup.sh

ride_display_start() {
  DISPLAY_ID="${1}"

  # if no XVFB is running, run XVFB
  if [[ $(ps aux | grep "[X]vfb ${DISPLAY_ID}") ]]; then
    echo "Xvfb already running for Display ${DISPLAY_ID}"
  else
    Xvfb "$DISPLAY_ID" -screen 0 1440x900x16 -ac -pn -noreset & 2> /dev/null
  fi

  # Start window manager
  export DISPLAY="${DISPLAY_ID}"
  WINDOW_MANAGER=${WINDOW_MANAGER:-'openbox'}
  nohup $WINDOW_MANAGER >/tmp/${WINDOW_MANAGER}.log & 2> /dev/null

  # remove ":" char from the display id
  DISPLAY_INT="${DISPLAY_ID:1}"

  # calculate vnc & novnc ports
  VNC_PORT=$(expr 5900 + "$DISPLAY_INT")
  NOVNC_PORT=$(expr 6080 + "$DISPLAY_INT")
  export VNC_PORT
  export NOVNC_PORT

  # launch VNC for DISPLAY
  nohup x11vnc -display "$DISPLAY" \
   -noxdamage -forever -localhost -rfbport "${VNC_PORT}" \
   -bg >/tmp/x11vnc-${DISPLAY_INT}.log & 2> /dev/null

  # launch NoVNC server
  BACKUP_PWD=$PWD
  cd /opt/novnc/utils
  nohup ./novnc_proxy \
   --vnc "localhost:${VNC_PORT}" \
   --listen "${NOVNC_PORT}" >/tmp/novnc-${DISPLAY_INT}.log & 2> /dev/null
  cd "${BACKUP_PWD}";
}
