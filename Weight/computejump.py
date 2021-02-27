#!/usr/bin/python3
from lxml import etree
import xml.etree.ElementTree as ET
import sys,os
from math import pi
from stoheader import  getstovalue,lastval
trajfile='Analyzes/Wtraj30.0.0.12.0.sto'
if len(sys.argv)>1:
	trajfile=sys.argv[1]
print("-----start fixing actuators.xml-----")
tofill = ET.parse('actuators.xml')
filler = ET.parse('results/colo_initial.osim')
fillerroot=filler.getroot()
tofillroot=tofill.getroot()
gotsprings=filler.findall('.//PathSpring')
print("removing all pathsprings from actuators.xml")
print("removing",tofillroot[0][3].get("name"));tofillroot[0].remove(tofillroot[0][3])
print("removing",tofillroot[0][3].get("name"));tofillroot[0].remove(tofillroot[0][3])
print("removing",tofillroot[0][3].get("name"));tofillroot[0].remove(tofillroot[0][3])
print("reading variables from traj")
spA=getstovalue(trajfile,'num_of_springs_ankle');stifA=float(spA)*5/(0.05*0.05*pi/2)
spH=getstovalue(trajfile,'num_of_springs_hip');stifH=float(spH)*5/(0.05*0.05*pi/2)
spK=getstovalue(trajfile,'num_of_springs_knee');stifK=float(spK)*5/(0.05*0.05*pi/2)
print("setting A/H/K to:",stifA,stifH,stifK)
m=getstovalue(trajfile,'total exo weight');weight=float(m);
allbody=fillerroot.findall('.//Body')
for b in allbody:
	if b.get("name")=="thigh_exo" or b.get("name")=="shank_exo":
		b.find("mass").text=str(weight/2)
print("changing mass to :",weight/2)
print("writing the wanalyze.osim")
filler.write("wanalyze.osim")

print("got:",spA,spH,spK,weight,"from",trajfile)
print("replacing stiffnesses values at  actuators.xml")
for i in range(3):
	if gotsprings[i].get("name")=="knee_spring":
		gotsprings[i].find("stiffness").text=str(stifK )
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
an2root[0].set("name",r[1][:-4].replace("traj",""))
print("relplacing controls_file states_file datafile")
an2root.find(".//controls_file").text=trajfile
an2root.find(".//states_file").text=trajfile
an2root.find(".//datafile").text="results/WGRF"+trajfile[14:-4]+".sto"
print("writing analyze2.xml as actuators")
an2.write("analyze2.xml")
os.system("opensim-cmd run-tool analyze2.xml&>/dev/null")
Y=lastval(trajfile[:-4].replace("traj","")+'_BodyKinematics_pos_global.sto',"center_of_mass_Y")
V=lastval(trajfile[:-4].replace("traj","")+'_BodyKinematics_vel_global.sto',"center_of_mass_Y")
Y=float(Y);V=float(V)
print("YV:",Y,V)
filej=getstovalue(trajfile,'jump');
print("j infile",filej)
print(spK,spH,spA,weight,Y+V*V/2/9.81)

#fil=sys.argv[1]
#treename=sys.argv[2]
#intree=ET.parse(fil)
#dom = xml.dom.minidom.parse(fil) 


