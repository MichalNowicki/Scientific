#!/bin/sh

for (( i=1,w=1; $i < 1422; i=i+5, w=w+1 )) ; do
	   
	   rgb=`ls rgb | head -n $i | tail -1`;
	   d=`ls depth | head -n $i | tail -1`;

       echo $d
     #  echo $d
     #  echo "---"
       
       cmd="./con depth/$d depth/new/$w.xml";
       eval $cmd 
       cmd2="cp rgb/$rgb rgb/new/$w.png";
       eval $cmd2 
done
