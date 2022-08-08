#!/bin/bash
fil='src/inpdata.txt'
n=120
#fil=inptry
sw=work;swn=1
#sw=stiff;swn=0
ang=90
sed -i "s/Dtraj0.0.*.equal/Dtraj0.0.$n.85.equal/" $fil
#sed -i "s/readGuessfile 1/readGuessfile 0/" $fil
#sed -i 's/work/stiff/;s/stiffOrWork 1/stiffOrWork 0/' $fil
#sed -i "s/startAngle.*/startAngle $ang/" $fil
#egrep 'stif|read|Ang' inptry
#echo ./ccolo
sed -i "s/equal_.*.sto/equal_$sw.sto/" $fil
sed -i "s/readGuessfile 0/readGuessfile 1/" $fil
for i in $(seq 1 18);do
sed -i "s/0.0.$n.*e/0.0.$n.$ang.e/" $fil
	ang=$(($ang-5))

sed -i "s/stiffOrWork ./stiffOrWork ${swn}/" $fil
sed -i "s/startAngle.*/startAngle $ang/" $fil
egrep "$sw|read|Ang" $fil
 ./ccolo
done
