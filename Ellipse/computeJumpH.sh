#!/bin/bash
#get knee hip ankle springs
function onejump {
unit=$(grep At90 $1 |cut -d"=" -f2)
n1=$(echo $1|cut -dj -f2|cut -d. -f1)
n2=$(echo $1|cut -dj -f2|cut -d. -f2)
n3=$(echo $1|cut -dj -f2|cut -d. -f3)
s1=$(bc <<<"scale=5;$n1*$unit*2/3.1415/0.05/0.05")
s2=$(bc <<<"scale=5;$n2*$unit*2/3.1415/0.05/0.05")
s3=$(bc <<<"scale=5;$n3*$unit*2/3.1415/0.05/0.05")
xmlstarlet ed -L -u  '//ForceSet/objects/NonlinearSpring[@name="knee_nlspring"]/stiffness' \
	-v  $s1 \
	 -u '//ForceSet/objects/PathSpring[@name="hip_spring"]/stiffness' -v  $s2 \
	  -u '//ForceSet/objects/PathSpring[@name="ankle_spring"]/stiffness' -v  $s3 actuators.xml
sed -i  "s|../Analyzes/Etraj.*sto|$1|g" analyze2.xml
sed -i  "s|EHA[0-9\.]*|EHA$n1.$n2.$n3|g" analyze2.xml
 ~/MOCOBIN/opensim-core/bin/opensim-cmd run-tool ./analyze2.xml &>/dev/null
 ypos=$(tail -1 Analyzes/EHA$n1.$n2.${n3}_BodyKinematics_pos_global.sto |cut -f 45)
 yvel=$(tail -1 Analyzes/EHA$n1.$n2.${n3}_BodyKinematics_vel_global.sto |cut -f 45)
 read num1 num2 num3<<<${fil//[^0-9]/ }
 echo -ne $num1 $num2 $num3  " "
 python -c "print($ypos+$yvel*$yvel/2/9.81)"
 }
function allreadyComputed {
n1=$(echo $1|cut -dj -f2|cut -d. -f1)
n2=$(echo $1|cut -dj -f2|cut -d. -f2)
n3=$(echo $1|cut -dj -f2|cut -d. -f3)
posfile=../Analyzes/EHA$n1.$n2.${n3}_BodyKinematics_pos_global.sto
velfile=../Analyzes/EHA$n1.$n2.${n3}_BodyKinematics_vel_global.sto
if [ -a $posfile ];then
	if [ $fil -nt $posfile ];then
	       onejump $fil
       	else
	 ypos=$(tail -1 $posfile |cut -f 45)
	 yvel=$(tail -1 $velfile |cut -f 45)
	 read num1 num2 num3<<<${fil//[^0-9]/ }
	 echo -ne E $num1 $num2 $num3  " "
	 python -c "print($ypos+$yvel*$yvel/2/9.81)"
 	fi
else
	 onejump $fil
fi
}
function some {
for fil in $1;do 
	allreadyComputed $fil
done
echo
}
if [ $# -eq 0 ]; then
#some 'results/traj*.0.0.sto'
#some 'results/traj0.*.0.sto'
#some 'results/traj0.0.*.sto'
some '../Analyzes/Etraj*.sto'
fi
if [ $# = 3 ];then
	fil="../Analyzes/Etraj$1.$2.$3.sto"
	if [ -a $fil ];then
		onejump $fil
	fi
fi
if [ $# = 1 ];then
	fil=$1
  allreadyComputed $fil
fi
