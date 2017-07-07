#!/bin/bash
#This cron job should be run periodically - every minute should be sufficient. 
#This script will check if the PIDFILE given below exists and will use the
#contained process ID to search for the corresponding process. If the process
#is not found, it will be restarted and the new PID will be placed in PIDFILE 

#TODO: We need to check whether the controller is operating correctly even if
#      the process is running. The controller may be hanging.
export HOME=/home/pi
export PROJECT_HOME=$HOME/projects
export ACE_ROOT=$PROJECT_HOME/ace/ACE_wrappers
export MADARA_ROOT=$PROJECT_HOME/madara
export GAMS_ROOT=$PROJECT_HOME/gams
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACE_ROOT/lib:$MADARA_ROOT/lib:$GAMS_ROOT/lib
export PATH=$ACE_ROOT/bin:$MADARA_ROOT/bin:$GAMS_ROOT/bin:$PATH


mkdir -p "$HOME/tmp"
PIDFILE="$HOME/tmp/boat_controller.pid"
CMD="/home/pi/gams_server/custom_controller -i 0 --madara-level 1 --gams-level 1 --broadcast 192.168.1.255:15000"

if [ -e "${PIDFILE}" ] && (ps -u $(whoami) -opid= |
                           grep -P "^\s*$(cat ${PIDFILE})$" &> /dev/null); then
  echo "Already running." >> $HOME/tmp/boat_controller.log
  exit 99
fi

echo "Restarting" >> $HOME/tmp/boat_controller.log 
#$CMD >> $HOME/tmp/boat_controller.log &
$CMD &

echo $! > "${PIDFILE}"
chmod 644 "${PIDFILE}"

