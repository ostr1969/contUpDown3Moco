#!/usr/bin/python3
import xml.etree.ElementTree as ET
import sys,os
from math import pi
from stoheader import  getstovalue,lastval
trajfile='Analyzes/Etraj30.0.0.0.055.sto'
if len(sys.argv)>1:
	trajfile=sys.argv[1]
	print("computing on",sys.argv[1])
print("-----start fixing actuators.xml-----")
ellip=getstovalue(trajfile,'ellipticb');ellip=ellip[:-3]
print("reading variables from traj")
spA=getstovalue(trajfile,'num_of_springs_ankle');stifA=float(spA)*5/(0.05*0.05*pi/2)
spH=getstovalue(trajfile,'num_of_springs_hip');stifH=float(spH)*5/(0.05*0.05*pi/2)
spK=getstovalue(trajfile,'num_of_springs_knee');stifK=float(spK)*5/(0.05*0.05*pi/2)
print(spK,spH,spA,ellip)
osimfilename="src/ellipse"+ellip+".osim"
tofill = ET.parse('actuators.xml')
filler = ET.parse(osimfilename)
tofillroot=tofill.getroot()
gotsprings=filler.findall('.//PathSpring')
gotNL=filler.findall('.//NonlinearSpring')
print("removing nonlinear springs from actuators.xml")
print("removing",tofillroot[0][0].get("name"))
tofillroot[0].remove(tofillroot[0][0])
print("removing",tofillroot[0][3].get("name"))
tofillroot[0].remove(tofillroot[0][3])
print("removing",tofillroot[0][3].get("name"))
tofillroot[0].remove(tofillroot[0][3])
print("setting A/H/K to:",stifA,stifH,stifK)

print("replacing stiffnesses values at  actuators.xml")
print("replacing", gotsprings[1].get("name"),"to", stifH)
gotsprings[1].find("stiffness").text=str(stifH )
print("replacing",gotsprings[2].get("name"),"to", stifA)
gotsprings[2].find("stiffness").text=str(stifA) 
print("replacing",gotNL[0].get("name"),"to", stifK)
gotNL[0].find("stiffness").text=str(stifK )
tofillroot[0].append(gotsprings[1])
tofillroot[0].append(gotsprings[2])
tofillroot[0].append(gotNL[0])
print("writing newactuators.xml as actuators")
tofill.write("newactuators.xml")
print("-----start fixing analyze2.xml-----")

an2 = ET.parse('analyze2.xml')
an2root=an2.getroot()
print("setting analyis name")
r=trajfile.split("/")
an2root[0].set("name","EHA"+r[1][5:-4])
print("relplacing controls_file states_file datafile")
an2root.find(".//controls_file").text=trajfile
an2root.find(".//states_file").text=trajfile
an2root.find(".//datafile").text="results/EGRF"+trajfile[14:-4]+".sto"
an2root.find(".//model_file").text="src/ellipse"+ellip+".osim"
print("writing analyze2.xml ")
an2.write("analyze2.xml")
os.system("opensim-cmd run-tool analyze2.xml&>/dev/null")
print("query last point trajfrom",trajfile[:-4].replace("traj","HA"))
Y=lastval(trajfile[:-4].replace("traj","HA")+'_BodyKinematics_pos_global.sto',"center_of_mass_Y")
V=lastval(trajfile[:-4].replace("traj","HA")+'_BodyKinematics_vel_global.sto',"center_of_mass_Y")
Y=float(Y);V=float(V)
print("found Y and V",Y,V)
print(spK,spH,spA,ellip,Y+V*V/2/9.81)

#fil=sys.argv[1]
#treename=sys.argv[2]
#intree=ET.parse(fil)
#dom = xml.dom.minidom.parse(fil) 


