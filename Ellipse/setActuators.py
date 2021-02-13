#!/usr/bin/python
import xml.etree.ElementTree as ET
import sys
if len(sys.argv)==5:
  elps=float(sys.argv[1])	
  elps="{:.3f}".format(elps)
  infile="src/ellipse"+elps+".osim"
  sp1=sys.argv[2];sp2=sys.argv[3];sp3=sys.argv[4];
elif len(sys.argv)==4 :
  infile='src/ellipse0.060.osim'
  sp1=sys.argv[1];sp2=sys.argv[2];sp3=sys.argv[3];
else:
  infile='src/ellipse0.060.osim'
  sp1=20;sp2=1;sp3=2
intree=ET.parse(infile)
inroot=intree.getroot()
osimnl1=inroot.find('.//NonlinearSpring')
inElliptica=osimnl1[2].text
inEllipticb=osimnl1[3].text
x1_=osimnl1[4][0].text
y1_=osimnl1[4][1].text
x2_=osimnl1[5][0].text
y2_=osimnl1[5][1].text
outtree = ET.parse('actuators.xml')
outroot = outtree.getroot()
nl1=outroot[0].find('NonlinearSpring')
outElliptica=nl1[2].text
outEllipticb=nl1[3].text
#x1=nl1[4][0].text
#y1=nl1[4][1].text
#x2=nl1[5][0].text
#y2=nl1[5][1].text
nl1[4][0].text=x1_
nl1[4][1].text=y1_
nl1[5][0].text=x2_
nl1[5][1].text=y2_
nl1[2].text=inElliptica
nl1[3].text=inEllipticb
nl1[1].text=str(sp1)
outroot[0][4][2].text=str(sp2)
outroot[0][5][2].text=str(sp3)
outtree.write('new.xml')
