#! /bin/sh

echo "Starting rexarm driver monitor script"

while [[ `pgrep -f "./rexarm_example"` ]]
do
	while [[ ! `pgrep -f "./rexarm_driver"` ]]
	do
		$HOME/eecs467/bin/./rexarm_driver
	done
done


