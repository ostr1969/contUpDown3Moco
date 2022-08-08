#!/bin/bash
#get knee hip ankle springs
function onejump {
unit=$(grep At90 $1 |cut -d"=" -f2)
n1=$(echo $1|cut -dj -f2|cut -d. -f1)
n2=$(echo $1|cut -dj -f2|cut -d. -f2)
n3=$(echo $1|cut -dj -f2|cut -d. -f3)
s1=$(grep -A1 time $1 |cut -f26|tail -n1)
s2=$(grep -A1 time $1 |cut -f27|tail -n1)
s3=$(grep -A1 time $1 |cut -f28|tail -n1)
nn1=$(bc <<<"scale=2;${s1}/($unit*2/3.1415/0.05/0.05)")
nn2=$(bc <<<"scale=2;${s2}/($unit*2/3.1415/0.05/0.05)")
nn3=$(bc <<<"scale=2;${s3}/($unit*2/3.1415/0.05/0.05)")
t1=$(bc <<<"scale=2;${nn1}*$unit")
t2=$(bc <<<"scale=2;${nn2}*$unit")
t3=$(bc <<<"scale=2;${nn3}*$unit")
xmlstarlet ed -L -u  '//ForceSet/objects/PathSpring[@name="path_spring1"]/stiffness' -v  $s1 \
	 -u '//ForceSet/objects/PathSpring[@name="path_spring2"]/stiffness' -v  $s2 \
	  -u '//ForceSet/objects/PathSpring[@name="path_spring3"]/stiffness' -v  $s3 Aactuators.xml
sed -i  "s|Analyzes/Atraj.*sto|$1|g" Aanalyze.xml
sed -i  "s|AKHA[0-9\.]*|AKHA$n1.$n2.$n3|g" Aanalyze.xml
 ~/MOCOBIN/opensim-core/bin/opensim-cmd run-tool ./Aanalyze.xml &>/dev/null
 ypos=$(tail -1 Analyzes/AKHA$n1.$n2.${n3}_BodyKinematics_pos_global.sto |cut -f 45)
 yvel=$(tail -1 Analyzes/AKHA$n1.$n2.${n3}_BodyKinematics_vel_global.sto |cut -f 45)
 read num1 num2 num3<<<${fil//[^0-9]/ }
 echo -ne $num1 $num2 $num3 ":" $nn1 $nn2 $nn3  "|" $t1 $t2 $t3 "|j="
 python -c "print($ypos+$yvel*$yvel/2/9.81)"
 }
function some {
for fil in $1;do 
	onejump $fil
done
echo
}
if [ $# -eq 0 ]; then
#some 'results/traj*.0.0.sto'
#some 'results/traj0.*.0.sto'
#some 'results/traj0.0.*.sto'
some 'Analyzes/Atraj*.sto'
else
	fil="Analyzes/Atraj$1.$2.$3.sto"
	if [ -a $fil ];then
		onejump $fil
	fi
fi
