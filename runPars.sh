#!/bin/bash
function repl {
sed -i "s|i num_of_springs_knee.*|i num_of_springs_knee $1|g" src/inpdata.txt
sed -i "s|i num_of_springs_hip.*|i num_of_springs_hip $2|g" src/inpdata.txt
sed -i "s|i num_of_springs_ankle.*|i num_of_springs_ankle $3|g" src/inpdata.txt
}
cat $1| while read line
do
	n1=$(echo $line|cut -d" " -f1)
	n2=$(echo $line|cut -d" " -f2)
	n3=$(echo $line|cut -d" " -f3)
	repl $n1 $n2 $n3
	#while ./ccolo; [ $? != 0  ] ;do :;done
	./ccolo

done 
