#!/usr/bin/bash

while true ; do 
	echo ''
	echo '***************************'
	echo '*   Start build and test  *'
	echo '***************************'
	echo ''
	make -j12 > /dev/null && test/libmodbus-static-test 
	sleep 3  
done

