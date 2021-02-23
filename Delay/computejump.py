#!/usr/bin/python3
import xml.etree.ElementTree as ET
import sys,os
from math import pi
from stoheader import  getstovalue,lastval
trajfile='Analyzes/Dtraj30.0.0.10.equal_work.sto'
if len(sys.argv)>1:
	trajfile=sys.argv[1]
print("-----start fixing actuators.xml-----")
tofill = ET.parse('actuators.xml')
filler = ET.parse('con3springs.osim')
tofillroot=tofill.getroot()
gotsprings=filler.findall('.//PathSpring')
print("removing all pathsprings from actuators.xml")
tofillroot[0].remove(tofillroot[0][3])
tofillroot[0].remove(tofillroot[0][3])
tofillroot[0].remove(tofillroot[0][3])
print("reading variables from traj")
spA=getstovalue(trajfile,'num_of_springs_ankle');stifA=float(spA)*5/(0.05*0.05*pi/2)
spH=getstovalue(trajfile,'num_of_springs_hip');stifH=float(spH)*5/(0.05*0.05*pi/2)
spK=getstovalue(trajfile,'num_of_springs_knee');stifK=float(spK)*5/(0.05*0.05*pi/2)
print("setting A/H/K to:",stifA,stifH,stifK)
stiffOpt=getstovalue(trajfile,'stiffness opt');
m=getstovalue(trajfile,'rest angle');restAng=float(m);
if stiffOpt=="equal_work":
	L1=float(gotsprings[0].find("resting_length").text)
	dalfa2=140-restAng
	F1max=stifK*140*pi/180*0.05
	F2max=F1max*140/dalfa2
	k2=F2max/(0.05*dalfa2*pi/180)
	restL=L1+restAng*pi/180*0.05
	print("setting restlen,k of",gotsprings[0].get("name"),"to",restL,k2)
	gotsprings[0].find("resting_length").text=str(restL)

print("replacing stiffnesses values at  actuators.xml")
print("replacing knee stiff",stifA,"to",k2)
for i in range(3):
	if gotsprings[i].get("name")=="knee_spring":
		gotsprings[i].find("stiffness").text=str(k2 )
	if gotsprings[i].get("name")=="hip_spring":
		gotsprings[i].find("stiffness").text=str(stifH) 
	if gotsprings[i].get("name")=="ankle_spring":
		gotsprings[i].find("stiffness").text=str(stifA )
for r in gotsprings:
	tofillroot[0].append(r)
	print("appending:",r.attrib,"to actuators")
print("writing newactuators.xml as actuators")
tofill.write("newactuators.xml")
print("-----start fixing analyze2.xml-----")

an2 = ET.parse('analyze2.xml')
an2root=an2.getroot()
print("setting analyis name")
r=trajfile.split("/")
an2root[0].set("name",r[1][:-4])
print("relplacing controls_file states_file datafile")
an2root.find(".//controls_file").text=trajfile
an2root.find(".//states_file").text=trajfile
an2root.find(".//datafile").text="results/DGRF"+trajfile[14:-4]+".sto"
print("writing analyze2.xml as actuators")
an2.write("analyze2.xml")
os.system("opensim-cmd run-tool analyze2.xml&>/dev/null")
Y=lastval(trajfile[:-4]+'_BodyKinematics_pos_global.sto',"center_of_mass_Y")
V=lastval(trajfile[:-4]+'_BodyKinematics_vel_global.sto',"center_of_mass_Y")
Y=float(Y);V=float(V)
print(spK,spH,spA,restAng,stiffOpt,Y+V*V/2/9.81)

#fil=sys.argv[1]
#treename=sys.argv[2]
#intree=ET.parse(fil)
#dom = xml.dom.minidom.parse(fil) 


