#! /bin/sh

echo "Starting rexarm driver monitor script"

while [ `pgrep -f "./rexarm_example"` ]
do
	if [ ! `pgrep -f "./rexarm_driver"` ]
	then
		./rexarm_driver
	fi
done


