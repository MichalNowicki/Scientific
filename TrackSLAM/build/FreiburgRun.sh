#!/bin/sh

for (( i=1; $i < 25; i=i+1 )) ; do
	   
	   w=`expr $i + 1`;
	  
	   txt="./kinect WYN$i depth/new/$i.xml rgb/new/$i.png depth/new/$w.xml rgb/new/$w.png";
       eval $txt
done

