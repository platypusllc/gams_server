#!/bin/bash
#This cron job should be run periodically - every minute should be sufficient. 
#This script will check if the PIDFILE given below exists and will use the
#contained process ID to search for the corresponding process. If the process
#is not found, it will be restarted and the new PID will be placed in PIDFILE 

#TODO: We need to check whether the controller is operating correctly even if
#      the process is running. The controller may be hanging.
mkdir -p "$HOME/tmp"
PIDFILE="$HOME/tmp/boat_controller.pid"
CMD="/home/pi/gams_server/custom_controller -i 1 --madara-level 1 --gams-level 1 --broadcast 192.168.1.255:15000"

if [ -e "${PIDFILE}" ] && (ps -u $(whoami) -opid= |
                           grep -P "^\s*$(cat ${PIDFILE})$" &> /dev/null); then
  echo "Already running."
  exit 99
fi

echo "Restarting" >> $HOME/tmp/boat_controller.log 
$CMD >> $HOME/tmp/boat_controller.log &
#$CMD

echo $! > "${PIDFILE}"
chmod 644 "${PIDFILE}"

