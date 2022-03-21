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

source common.sh
source evaluation_script.sh

clearUpCatkinWorkspace() {
    rm -rf ${OUTPUT_PATH}
    mkdir ${OUTPUT_PATH}

    rm -rf ${PROJECT_PATH}/world
    mkdir ${PROJECT_PATH}/world

    cp worlds/$1/input_points.yaml ${PROJECT_PATH}/world
    cp -r worlds/$1/map ${PROJECT_PATH}/world
}

prompt_deletion() {
    RED='\033[0;31m'
    NC='\033[0m' # No Color
    bold=$(tput bold)
    normal=$(tput sgr0)

    printf "${RED}${bold}****** !!!!! WARNING !!!!!! *****${normal}\n\n" 
    printf "${RED}This script will ${bold}DELETE YOUR catkin_ws/src/world DIRECTORY${normal} ${RED}!
You should make sure that you have git committed all changes and pushed them to GitLab otherwise any un-committed
files/changes in that directory will be lost permanently.${normal}\n\n "

    while true; do
        read -p "Type 'delete catkin_ws/src/world' to continue or 'stop' to stop: " userInput
        case ${userInput} in
            "delete catkin_ws/src/world" ) break;;
            "stop" ) echo 'Stopping...'; exit;;
            * ) echo "Please type exactly 'delete catkin_ws/src/world' to continue or 'stop' to stop.";;
        esac
    done
}

prompt_deletion
declare -a WORLDS=('world1' 'world2')

DEBUG=0
if [[ $* == *--debug* ]]; then
    DEBUG=1
fi

FENG=0
if [[ $* == *--feng* ]]; then
    FENG=1
fi

for world in "${WORLDS[@]}"
do
    echo 'Evaluating ' $world
    clearUpCatkinWorkspace $world
    evaluate $world
    printf '\n\n'
done

