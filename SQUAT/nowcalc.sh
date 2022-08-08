#!/bin/bash
sed -i "s/traj.*.sto/traj$1.$2.$3.sto/;
0,/knee/s/knee.*/knee $4/;
0,/hip/s/hip.*/hip $5/;
0,/ankle/s/ankle.*/ankle $6/" src/inpdata.txt
./ccolo
