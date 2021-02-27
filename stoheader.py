#!/bin/python3
import sys
import glob
def getstovalue(filn,var):
	with open(filn) as f:
		lines=f.readlines()
	for line in lines:
		if line=="endheader\n":
			break      
		if line.find(var)>-1:
			res=line.split("=")
			return res[1].replace("\n","")
	return "-"
def lastline(fil):
	with open(fil, "r") as file:
		first_line = file.readline()
		for last_lin in file:
			pass
	return last_lin
def printcol(fil,txt):
	readtitles=0;cols=[0];startprint=0
	with open(fil, "r") as file:
		for last_lin in file:
			if last_lin=="endheader\n":
				readtitles=1
				continue
			if readtitles==1:
				tit=last_lin.split("\t")
				print(tit[0],end=" ")
				for count,v in enumerate(tit):
					if v.find(txt)>-1:
						cols.append(count)
						print(v,end=" ")
				print()
				readtitles=0;startprint=1;continue
			if startprint==1:
				tit=last_lin.split("\t")
				for i in cols:
					pass
					print(tit[i],end=" "),
				print()
def lastval(fil,txt):
	readtitles=0;cols=[];startprint=0
	with open(fil, "r") as file:
		for last_lin in file:
			if last_lin=="endheader\n":
				readtitles=1
				continue
			if readtitles==1:
				tit=last_lin.split("\t")
				#print(tit[0],end=" ")
				for count,v in enumerate(tit):
					if v.find(txt)>-1:
						cols.append(count)
				readtitles=0;startprint=1;continue
			if startprint==1:
				pass   
			tit=last_lin.split("\t")
		if len(cols)>1:
			print("error:more then one column found");sys.exit()
		if len(cols)==0:
			print("error:no column found with name",txt);sys.exit()
		return(tit[cols[0]])
def printtitles(fil):
	readtitles=0;cols=[];startprint=0
	with open(fil, "r") as file:
		for last_lin in file:
			if last_lin=="endheader\n":
				readtitles=1
				continue
			if readtitles==1:
				tit=last_lin.split("\t")
				for count,v in enumerate(tit):
					print(count,v)
				readtitles=0;break
def getvarsInline(filn):
	K=getstovalue(filn,"num_of_springs_knee")
	H=getstovalue(filn,"num_of_springs_hip")
	A=getstovalue(filn,"num_of_springs_ankle")
	J=getstovalue(filn,"jump")
	S=getstovalue(filn,"stif_multiplier")
	I=getstovalue(filn,"initial_angle")
	W=getstovalue(filn,"total exo weight")
	E=getstovalue(filn,"ellipticb")
	Y=getstovalue(filn,"rest angle")
	F=getstovalue(filn,"stiffness opt")
	print(K,H,A,J,S,I,W,E,Y,F)
def allresults():
	tr=glob.glob('Analyzes/*traj*')
	for f in tr:
		getvarsInline(f)

if __name__ == "__main__":
	#p=getstovalue('Analyzes/Otraj0.0.30.92.15.sto','timediv')
	#p=lastline('Analyzes/Otraj0.0.30.92.15.sto')
	#p=printcol( "Analyzes/traj48.48.0.sto", "aforce.HalfSpace.torque.Y")
	#p=printtitles('Analyzes/Otraj0.0.30.92.15_BodyKinematics_pos_global.sto')
	#p=lastval('Analyzes/Otraj0.0.30.92.15_BodyKinematics_pos_global.sto',"center_of_mass_Y")
	#getvarsInline("Analyzes/traj48.48.0.sto")
	allresults()
