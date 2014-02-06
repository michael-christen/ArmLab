#! /bin/sh

if [[ `find  ../../bin -name driver_monitor.sh` ]]
then
	echo "driver_monitor.sh found in bin"
else
	echo "placing driver_monitor.sh in bin"
	cp driver_monitor.sh ../../bin
fi
