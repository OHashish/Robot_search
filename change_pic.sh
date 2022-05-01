#!/bin/bash
echo 'CHANGES THE CHARACTER IMAGE, args: first 2  letters of char'

case $1 in
	sc)
	 echo 'Scarlet'
	 cp ~/catkin_ws/src/group_project/cluedo_images/scarlet.png ~/.gazebo/models/cluedo_character/materials/textures/Cluedo_character.png
	;;
	pl)
	 echo 'Plum'
	 cp ~/catkin_ws/src/group_project/cluedo_images/plum.png ~/.gazebo/models/cluedo_character/materials/textures/Cluedo_character.png
	;;

	mu)
	 echo 'Mustard'
	 cp ~/catkin_ws/src/group_project/cluedo_images/mustard.png ~/.gazebo/models/cluedo_character/materials/textures/Cluedo_character.png
	;;

	pe)
	 echo 'Peacock'
	 cp ~/catkin_ws/src/group_project/cluedo_images/peacock.png ~/.gazebo/models/cluedo_character/materials/textures/Cluedo_character.png
	;;

	*)
	 echo 'error';;
esac
	
