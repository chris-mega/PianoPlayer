#!/bin/bash

########################
# first argument is:
# 0 Billie Jean
# 1 knocking on heavens door
# 2 cantonese song

# second argument (only required when running concert):
# 0 no udp waiting
# 1 udp waiting enabled

# second argument (only required when running sound check):
# 0 no ik
# 1 ik enabled
##################################


#make sure 3 arguments were passed
if [ $# -lt 1 ] || [ $# -gt 2 ]; then
	echo "Need to pass 2 or 1 arguments: $ drums.sh [song/billie jean] [udp]"
	exit
fi

song="song:="
udp="udp:="

chosen_command="roslaunch events piano_player.launch"

# first check song
if [ $1 == "0" ]; then
	echo "---Billie Jean selected."
	song=" ${song}billie "
elif [ $1 == "1" ]; then
	echo "---Knocking on selected."
	song=" ${song}knocking "
elif [ $1 == "2" ]; then
	echo "---Cantonese song selected"
	chosen_command="roslaunch events piano_player.launch"
	song=" ${song}beyond "
else
	echo "-------------Invalid song"
	exit
fi


#if $1 is 2 (billie jean) the command is ready
if [ $1 == "0" ]; then
	ik="ik:="
	if [ $2 == "0" ]; then
		echo "---IK not selected"
		ik=" ${ik}0 "
	elif [ $2 == "1" ]; then
		echo "---IK selected"
		ik=" ${ik}1 "
	else
		echo "-------------Invalid ik option"
		exit
	fi
	command_to_execute="${chosen_command} ${song} ${ik}"
	echo "Running command: "
	echo $command_to_execute
	read -p "Press enter to RUN!" unused
	eval $command_to_execute
fi


if [ $2 == "0" ]; then
        echo "---UDP waiting NOT selected"
        udp=" ${udp}0 "
elif [ $2 == "1" ]; then
        echo "---UDP waiting SELECTED"
        udp=" ${udp}1 "
else
        echo "-------------Invalid UDP Wait"
        exit
fi

#finally ready to execute the command

command_to_execute="${chosen_command} ${song} ${udp}"
echo "Running command: "
echo $command_to_execute
read -p "Press enter to RUN!" unused
eval $command_to_execute
