#! /bin/sh

if [[ `find  $HOME/eecs467/bin -name driver_monitor.sh` ]]
then
	echo "driver_monitor.sh found in bin"
else
	echo "placing driver_monitor.sh in bin"
	cp driver_monitor.sh $HOME/eecs467/bin
fi
