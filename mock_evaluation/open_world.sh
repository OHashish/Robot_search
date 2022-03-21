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

startGazebo() {
    export TURTLEBOT_GAZEBO_WORLD_FILE=${1}
    gazebo="roslaunch turtlebot_gazebo turtlebot_world.launch"
    if [ $DEBUG == 0 ]; then
        gazebo="${gazebo} > /dev/null 2>&1"
    fi
    if [ $FENG == 1 ]; then
        gazebo="vglrun ${gazebo}"
    fi
    eval "${gazebo} &"

    countdown 'Starting Gazebo...' '00:00:05'
}

loadLocalisation() {
    localisation="roslaunch $PROJECT_PATH/launch/simulated_localisation.launch map_file:=${1}"
    if [ $DEBUG == 0 ]; then
        localisation="${localisation} > /dev/null 2>&1"
    fi
    if [ $FENG == 1 ]; then
        localisation="vglrun ${localisation}"
    fi
    eval "${localisation} &"

    countdown 'Starting localisation module...' '00:00:15'
} 

launchRviz() {
    rviz="roslaunch turtlebot_rviz_launchers view_navigation.launch"
    if [ $DEBUG == 0 ]; then
        rviz="${rviz} > /dev/null 2>&1"
    fi
    if [ $FENG == 1 ]; then
        rviz="vglrun ${rviz}"
    fi
    eval "${rviz} &"

    countdown 'Starting RViz...' '00:00:05'
}

worldFile=$PWD/worlds/${1}/project.world
mapFile=$PWD/worlds/${1}/map/project_map.yaml

startGazebo ${worldFile}
loadLocalisation ${mapFile}
launchRviz
