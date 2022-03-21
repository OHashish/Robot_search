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

copyResults() {
    mkdir -p ${EVALUATION_FILE_PATH}/${1}

    if [[ -f ${OUTPUT_PATH}/cluedo_character.txt ]]; then
        cp ${OUTPUT_PATH}/cluedo_character.txt ${EVALUATION_FILE_PATH}/${1}
    else
        echo 'No text file exists.'
    fi

    if [[ -f ${OUTPUT_PATH}/cluedo_character.png ]]; then
        cp ${OUTPUT_PATH}/cluedo_character.png ${EVALUATION_FILE_PATH}/${1}
    else
        echo 'No image file exists.'
    fi
}

evaluate() {
    # Start all ROS tools
    source open_world.sh $1

    # Init robot pose; move robot around to localise; move robot to start
    # position.
    source ${PWD}/worlds/${1}/init_pose.sh; countdown 'Initialising robot pose...' '00:00:03'
    source ${PWD}/worlds/${1}/localise_pose.sh; countdown 'Localising robot ...' '00:00:15'
    source ${PWD}/worlds/${1}/start_pose.sh; countdown 'Moving robot to start position ...' '00:00:20'

    # Run main program and capture its process ID.
    rosrun group_project main.py &
    solutionPID=$!

    # Wait for max duration (5 minutes) for main program to complete.
    countdown 'main.py program running...' '00:05:00'

    # Kill the main script
    kill -9 $solutionPID
    echo 'main.py terminated.'

    # Kill all ROS software
    source kill_all.sh
    echo 'Killed all ROS tools'

    # Copy all output results for evaluation
    copyResults ${1}
    echo 'Output files copied'
    sleep 2
}
