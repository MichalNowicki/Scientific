#!/bin/sh

eval "python2 associate.py rgb.txt depth.txt > matched"
eval "rm matched2"
for (( i=1,w=1; $i < 1300; i=i+2, w=w+1 )) ; do
	   
	   rgb=`cat matched | cut -d" " -f 1 | head -n $i | tail -1`;
	   d=`cat matched | cut -d" " -f 3 | head -n $i | tail -1`;

       echo $d
     #  echo $d
     #  echo "---"
       
       eval "echo $d >> matched2"; 
       
       cmd="./con depth/$d.png depth/new/$w.xml";
       eval $cmd 
       cmd2="cp rgb/$rgb.png rgb/new/$w.png";
       eval $cmd2 
done
