#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

# `reboot` returns while shutdown is still in progress. Flashing AGNOS after a
# successful verify/swap would corrupt the slot that was just activated, so
# never continue once a reboot was requested.
function reboot_and_wait {
  sudo reboot
  sleep 300
}

function wait_if_rebooting {
  if [ "$(systemctl is-system-running 2> /dev/null)" = "stopping" ]; then
    sleep 300
  fi
}

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    # the branch we were switched from may have already swapped slots and requested the reboot
    wait_if_rebooting
    # AGNOS 19.x has no pyserial; updaters of newer branches install this shim when they switch to us
    if [ -d "$DIR/system/hardware/tici/pyserial_compat" ] && ! python3 -c "import serial" 2> /dev/null; then
      export PYTHONPATH="$DIR/system/hardware/tici/pyserial_compat${PYTHONPATH:+:$PYTHONPATH}"
    fi
    if $AGNOS_PY --verify $MANIFEST; then
      reboot_and_wait
    fi
    while true; do
      # our updater can't draw on AGNOS 20.4+ (incompatible raylib); the OS ships a matching one
      $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST || \
        { [ -x /usr/comma/updater ] && /usr/comma/updater $AGNOS_PY $MANIFEST; }
      # no updater could finish the job; flash and swap without UI
      wait_if_rebooting
      if $AGNOS_PY --swap $MANIFEST; then
        reboot_and_wait
      fi
    done
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
