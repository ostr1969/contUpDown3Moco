# contUpDown3Moco
#git commit -a -m "with initial set"
set src/input for the spring level
TorqueAt90 is each spring:now  is 5Nm
q0 unused
q1 q2 q3 pelvisx pelvisy are the initials
initial model writen modelfile=con3springs.osim
guessfile is toggled 0 or 1

computeJumpH.sh 0 30 0 is to cumpute allredy solved solution 
from results/GRF0.30.0.sto and  Analyzes/traj0.30.0.sto
it uses analyze2.xml actuators.xml

to run set of cases runpars [filename] where filename is list of tree numbers

main run is ./ccolo
costs in MocoJumpGoal.cpp are:
0: jump height
1: slide in x
2: tip at 0 all over the run

"colo" cpp files:
RegisterTypes_osimMocoJumpGoal.cpp  MocoJumpGoal.cpp colo.cpp
#include "console.h"
#include "additions.h"
#include "MocoJumpGoal.h"

delpactuator can use: 
updateDelpActuator(Model& osimModel,string actu,string filename,double activation,
                double deactivation,int qfac_dir,double maxvel)
to enter parameters and delp.txt files (angle dependency)
but its important to enter the curves in the osim file - done in build3cont.cpp
important for the post analysis

Limits are activated 5 deg before at the end of range
 
#static initial pose
the pelvis_rot is fixed before solving such that the com is abouve the tip
comang is fixed to pelrot
then the height is fixed to touch the ground
then height(coord2) is fixed again iteratively until force is 784
the initial pose is tranfered to default
the initial activations are transfer to the default activation
and olso to the initial at the collocations
if the initial acts are not -1 to 1 then exit

after solution , the height is computed by:
	analyze.xml, con3springs.osim,results/colo_forces.sto,results/colo_states.sto
and many parameters are injected to trajXXX.sto



"build" files:
build3cont.cpp
#include "additions.h"
#include "readx_y.h"
readx_y can read text file with x and y colums and return x y and num of pairs
SimmSpline take those pairs and convert to spline
then after DelpActivator is built setTorqueAngleCurve()  can insert this spline to the osim model
the file exported is con3springs.osim

other optinos:
OnlyAnkle - modify ankle rest angle and stiifness multiplier
Weight - modify knee total weight
Init - modify knee initial tou while maintaining work (obselete)
Delay - modify  rest angle while maintaining end force or total work
Ellipse - changing knee wrap to ellipse with a=0.05 and var b
