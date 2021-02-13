#!/bin/bash
#get knee hip ankle springs
function onejump {
unit=$(grep At90 $1 |cut -d"=" -f2)
e4=$1
n1=$(echo $1|cut -dj -f2|cut -d. -f1)
n2=$(echo $1|cut -dj -f2|cut -d. -f2)
n3=$(echo $1|cut -dj -f2|cut -d. -f3)
e4=${e4: -9:-4}
s1=$(bc <<<"scale=5;$n1*$unit*2/3.1415/0.05/0.05")
s2=$(bc <<<"scale=5;$n2*$unit*2/3.1415/0.05/0.05")
s3=$(bc <<<"scale=5;$n3*$unit*2/3.1415/0.05/0.05")
./setActuators.py $e4 $s1 $s2 $s3
sed -i  "s|Analyzes/Etraj.*sto|$1|g" analyze2.xml
sed -i  "s|EHA[0-9\.]*|EHA$n1.$n2.$n3.$e4|g" analyze2.xml
sed -i  "s|EGRF[0-9\.]*|EGRF$n1.$n2.$n3.$e4.|g" analyze2.xml
 ~/MOCOBIN/opensim-core/bin/opensim-cmd run-tool ./analyze2.xml &>/dev/null
 ypos=$(tail -1 Analyzes/EHA$n1.$n2.$n3.${e4}_BodyKinematics_pos_global.sto|cut -f 45)
 yvel=$(tail -1 Analyzes/EHA$n1.$n2.$n3.${e4}_BodyKinematics_vel_global.sto |cut -f 45)
 read num1 num2 num3<<<${fil//[^0-9]/ }
 echo -ne $num1 $num2 $num3  " "
 python -c "print($ypos+$yvel*$yvel/2/9.81)"
 }
function allreadyComputed {
e4=$1
n1=$(echo $1|cut -dj -f2|cut -d. -f1)
n2=$(echo $1|cut -dj -f2|cut -d. -f2)
n3=$(echo $1|cut -dj -f2|cut -d. -f3)
e4=${e4: -9:-4}
posfile=Analyzes/EHA$n1.$n2.$n3.${e4}_BodyKinematics_pos_global.sto
velfile=Analyzes/EHA$n1.$n2.$n3.${e4}_BodyKinematics_vel_global.sto
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
some 'Analyzes/Etraj*.sto'
fi
if [ $# = 4 ];then
	fil="Analyzes/Etraj$2.$3.$4.$1.sto"
	if [ -a $fil ];then
		onejump $fil
	fi
fi
if [ $# = 1 ];then
	fil=$1
  allreadyComputed $fil
fi
