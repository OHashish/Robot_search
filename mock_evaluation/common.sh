# Copyright (C) 2022 University of Leeds
# 
# Author: Rafael Papallas, please contact: r.papallas@leeds.ac.uk
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
# 
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
# details.
# 
# You should have received a copy of the GNU General Public License along with
# this program. If not, see http://www.gnu.org/licenses/.

countdown() (
    # Reference/credit: https://www.cyberciti.biz/faq/how-to-display-countdown-timer-in-bash-shell-script-running-on-linuxunix/
    IFS=:
    set -- $*
    secs=$(( ${2#0} * 3600 + ${3#0} * 60 + ${4#0} ))
    while [ $secs -gt 0 ]
    do
        sleep 1 &
        printf "\r ${1} %02d:%02d:%02d" $((secs/3600)) $(( (secs/60)%60)) $((secs%60))
        secs=$(( $secs - 1 ))
        wait
    done
    echo
)

CATKIN_WS=${HOME}/catkin_ws
PROJECT_PATH=${CATKIN_WS}/src/group_project
OUTPUT_PATH=${PROJECT_PATH}/output
EVALUATION_FILE_PATH=${HOME}/Desktop/comp3631_evaluation_files
source /opt/ros/indigo/setup.bash
source $CATKIN_WS/devel/setup.bash
