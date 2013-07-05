#!/bin/sh

for (( i=1; $i <= 6; i++ )) ; do
	   w=`expr $i - 1`;
       txt="./kinect log$w frameD_$w.xml frameBGR_$w.png frameD_$i.xml frameBGR_$i.png"

       eval $txt
done
