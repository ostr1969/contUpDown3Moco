
/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application acts as an example for utilizing the 
 *  ControllabeSpring actuator.
 *
 * compare this code to the matlab code compareRBDL4 in :
 * C:\Users\ostr\Google Drive (ostr@post.bgu.ac.il)\PHD\C++\RBDL
 * activate ./myactuator 0.25 and look at the last line of
 * 4link_states_degrees.sto for the angle and velocities
 * the initial angles are read from  T0_0_140_0.txt(read the remark below
 * this file changed from the file in windows dir)
 * the torques are read from tou0_140_0.txt. 
*/
//==============================================================================
//==============================================================================
//#include "PistonActuator.h"
//#include "ControllableSpring.h"
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/DelpActuator.h>
#include "OpenSim/Common/STOFileAdapter.h"
#define BUILD 1
#include "additions.h"
#include <OpenSim/Common/readx_y.h>
//#include "myActuatorPowerProbe.cpp"
//#include "myForceSet.cpp"
//#include "DelpActuator.h"
using std::string;
#define MAXN 1000
using namespace OpenSim;
using namespace SimTK;
using namespace std::chrono;
using  std::cout;
using  std::endl;
int main(int argc, char *argv[])
{
try
{

        // Create a new OpenSim model.
        Model osimModel;
        osimModel.setName("4link3springCont");
        osimModel.setAuthors("barak ostraich");

            
        // Get the ground body.
        Ground& ground = osimModel.updGround();
        ground.attachGeometry(new Mesh("checkered_floor.vtp"));

        // Create first linkage body.
        double footMass = 2.32 , footLength = 0.2084, linkageDiameter = 0.02;
        double shankMass = 7.44 , shankLength = 0.4182;
        double thighMass = 16   , thighLength = 0.4165;
        double HATMass = 54.24, HATLength = 0.799 ;
        
        Vec3 linkageMassCenter1(0.,footLength-0.1292   ,0.);
        Vec3 linkageMassCenter2(0.,shankLength-0.18108   ,0.);
        Vec3 linkageMassCenter3(0.,thighLength-0.18034   ,0.);
        Vec3 linkageMassCenter4(0.,HATLength-0.5   ,0.);
        double exothick=0.015;
        Vec3 exocenter(0.,exothick/2,0.);

        OpenSim::Body* foot_body = new OpenSim::Body("foot", footMass,
                linkageMassCenter1, Inertia(0.1,0.1,0.03495,0.,0.,0.));
        OpenSim::Body* shank_body = new OpenSim::Body("shank", shankMass,
                linkageMassCenter2, Inertia(0.1,0.1,0.11867,0.,0.,0.));
        OpenSim::Body* thigh_body = new OpenSim::Body("thigh", thighMass,
                linkageMassCenter3, Inertia(0.2,0.1,0.28957 ,0.,0.,0.));
        OpenSim::Body* HAT_body = new OpenSim::Body("HAT", HATMass,
                linkageMassCenter4, Inertia(5.1,5.1,8.518,0.,0.,0.));//1.48




        //create z vector for all joints 
        Rotation R = Rotation(-Pi/2, ZAxis); 
        // Graphical representation of foot.
        Sphere sphere(0.02);
	Ellipsoid head(.15/2,.25/2,.15/2);
        //Sphere psphere(0.01);
        //psphere.upd_Appearance().set_color(SimTK::Vec3(1.0, 0.0, 0.0));
        Cylinder exo(.05, exothick);
        double exoMassTop=0.434*2,exoCmTop=0.134; //toppart mass and dist from knee
        double exoMassBot=0.451*2,exoCmBot=0.091; //toppart mass and dist from knee
	double exoI_Top=4.5*2/1000, exoI_Bot=6.9*2/1000; //moment of inertia kg*m*m for two
	Brick brick(Vec3(.05/2, 0.01, .01));
	
        Cylinder cyl1(linkageDiameter/2, footLength/2);
        Frame* cyl1Frame = new PhysicalOffsetFrame(*foot_body, 
            Transform(Vec3(0.0, footLength/2 , 0.0)));
        cyl1Frame->setName("Cyl1_frame");
        cyl1Frame->attachGeometry(cyl1.clone());
        //cyl1Frame->attachGeometry(exo.clone());
        // cyl1Frame->attachGeometry(new Mesh("foot.vtp"));
        Frame* b1Frame = new PhysicalOffsetFrame(*foot_body,
            Transform(Vec3(-0.05/2, footLength-.05, 0.)));
	b1Frame->attachGeometry(brick.clone());
        osimModel.addComponent(b1Frame);
        osimModel.addComponent(cyl1Frame);

        // Create shank body.
        Cylinder cyl2(linkageDiameter/2, shankLength/2);
        Frame* cyl2Frame = new PhysicalOffsetFrame(*shank_body,
            Transform(Vec3(0.0, shankLength/2 , 0.0)));
        cyl2Frame->setName("Cyl2_frame");
        cyl2Frame->attachGeometry(cyl2.clone());
        //cyl2Frame->attachGeometry(exo.clone());//cyl on shank(end of exo)

        Frame* b2Frame = new PhysicalOffsetFrame(*shank_body,
            Transform(Vec3(0.05/2, shankLength-.05, 0.)));
	//b2Frame->attachGeometry(brick.clone());//small cube on shank
        osimModel.addComponent(b2Frame);
        osimModel.addComponent(cyl2Frame);
        // Create thigh body.
        thigh_body->attachGeometry(sphere.clone());
        Cylinder cyl3(linkageDiameter/2, thighLength/2);
        Frame* cyl3Frame = new PhysicalOffsetFrame(*thigh_body,
            Transform(Vec3(0.0, thighLength/2 , 0.0)));
        cyl3Frame->setName("Cyl3_frame");
        cyl3Frame->attachGeometry(cyl3.clone());
        //cyl3Frame->attachGeometry(exo.clone());
        //cyl3Frame->attachGeometry(new Mesh("femur_r.vtp"));
        Frame* b3Frame = new PhysicalOffsetFrame(*thigh_body,
            Transform(Vec3(-0.05/2, thighLength-.05, 0.)));
	b3Frame->attachGeometry(brick.clone());
        osimModel.addComponent(b3Frame);
        osimModel.addComponent(cyl3Frame);
        // Create HAT body.
        HAT_body->attachGeometry(sphere.clone());
        Cylinder cyl4(linkageDiameter/2, HATLength/2);
        Frame* cyl4Frame = new PhysicalOffsetFrame(*HAT_body,
            Transform(Vec3(0.0, HATLength / 2.0, 0.0)));
        Frame* exo4Frame = new PhysicalOffsetFrame(*HAT_body,
            Transform(Vec3(0.0, 0.2, 0.0)));
        Frame* headFrame = new PhysicalOffsetFrame(*HAT_body,
            Transform(Vec3(0.0, HATLength, 0.0)));
	headFrame->attachGeometry(head.clone());
        cyl4Frame->setName("Cyl4_frame");
        cyl4Frame->attachGeometry(cyl4.clone());
        exo4Frame->attachGeometry(exo.clone());
        osimModel.addComponent(cyl4Frame);
        osimModel.addComponent(headFrame);
        osimModel.addComponent(exo4Frame);
	//create exoskeleton
	//I=m*r^2  r=sqrt(I/m)
	double exoRogTop=sqrt(exoI_Top/exoMassTop);//radi of giration
	double exoRogBot=sqrt(exoI_Bot/exoMassBot);//radi of giration bot
        exoMassTop=1;exoMassBot=1;double ITop=exoMassTop*exoRogTop*exoRogTop;
				  double IBot=exoMassBot*exoRogBot*exoRogBot;
	cout<<"exomass(two legs):"<<exoMassTop+exoMassBot<<endl;
	cout<<"exoI:"<<ITop<<","<<IBot<<endl;
        OpenSim::Body* thighExoBody = new OpenSim::Body("thigh_exo",exoMassTop ,
                Vec3(0,exoCmTop,0), Inertia(1.1,1.1,ITop,0.,0.,0.));
        OpenSim::Body* shankExoBody = new OpenSim::Body("shank_exo", exoMassBot,
                Vec3(0,exoCmBot,0), Inertia(1.1,1.1,IBot,0.,0.,0.));
       osimModel.addBody(thighExoBody);osimModel.addBody(shankExoBody);
	WeldJoint* weldt =
       new WeldJoint("weldtop", *thigh_body, Vec3(0, 0.0,0),Vec3(0),
		       *thighExoBody, Vec3(0), Vec3(0));
               osimModel.addJoint(weldt);
	WeldJoint* weldb =
       new WeldJoint("weldbot",*shank_body, Vec3(0,shankLength,0),Vec3(0,0,Pi),
		       *shankExoBody,Vec3(0),Vec3(0));
               osimModel.addJoint(weldb);
	       //top exo drawing
	Brick tbrick(Vec3(.01, exoCmTop, .01));
        Frame* TFrame1 = new PhysicalOffsetFrame(*thigh_body,
            Transform(Vec3(0, exoCmTop, 0.05)));
	TFrame1->attachGeometry(tbrick.clone());
        osimModel.addComponent(TFrame1);
        Frame* TFrame2 = new PhysicalOffsetFrame(*thigh_body,
            Transform(Vec3(0, exoCmTop, -0.05)));
	TFrame2->attachGeometry(tbrick.clone());
        osimModel.addComponent(TFrame2);
	//bot exo drawing
	Brick bbrick(Vec3(.01, exoCmBot, .01));
        Frame* BFrame1 = new PhysicalOffsetFrame(*shankExoBody,
            Transform(Vec3(0, exoCmBot, 0.05)));
	BFrame1->attachGeometry(bbrick.clone());
        osimModel.addComponent(BFrame1);
        Frame* BFrame2 = new PhysicalOffsetFrame(*shankExoBody,
            Transform(Vec3(0, exoCmBot, -0.05)));
	BFrame2->attachGeometry(bbrick.clone());
        osimModel.addComponent(BFrame2);
	
//create markers

    	Marker*m0 = new Marker();
    	m0->setName("m0");
    	m0->setParentFrame(*HAT_body);
    	m0->set_location(SimTK::Vec3(-0.05,0,0));
     	osimModel.addMarker(m0);
    	Marker*m1 = new Marker();
    	m1->setName("m1");
    	m1->setParentFrame(*thigh_body);
    	m1->set_location(SimTK::Vec3(-0.05,thighLength,0));
     	osimModel.addMarker(m1);
    	Marker*m2 = new Marker();
    	m2->setName("m2");
    	m2->setParentFrame(*thigh_body);
    	m2->set_location(SimTK::Vec3(0.05,0,0));
     	osimModel.addMarker(m2);
    	Marker*m3 = new Marker();
    	m3->setName("m3");
    	m3->setParentFrame(*shank_body);
    	m3->set_location(SimTK::Vec3(0.05,shankLength,0));
     	osimModel.addMarker(m3);
    	Marker*m4 = new Marker();
    	m4->setName("m4");
    	m4->setParentFrame(*shank_body);
    	m4->set_location(SimTK::Vec3(-0.05,0,0));
     	osimModel.addMarker(m4);
    	Marker*m5 = new Marker();
    	m5->setName("m5");
    	m5->setParentFrame(*foot_body);
    	m5->set_location(SimTK::Vec3(-0.05,footLength,0));
     	osimModel.addMarker(m5);
        Marker *tipmarker=new Marker("tipm",*foot_body,Vec3(0));
	osimModel.addMarker(tipmarker);


        // Create 1 degree-of-freedom pin joints between the bodies to create a
        // kinematic chain from ground through the block.
        Vec3 orientationInGround(0.);
        Vec3 locationInGround(0.);
        Vec3 locationInParent1(0.0, footLength, 0.0);
        Vec3 locationInParent2(0.0, shankLength, 0.0);
        Vec3 locationInParent3(0.0, thighLength, 0.0);
        Vec3 locationInParent4(0.0, HATLength, 0.0);
        Vec3 orientationInChild(0.);
        Vec3 locationInChild(0.);

        PinJoint* tip   = new PinJoint("tip",
                ground, locationInGround, orientationInGround,
                *foot_body, locationInChild, orientationInChild);

        PinJoint* ankle = new PinJoint("ankle",
                *foot_body, locationInParent1, orientationInChild,
                *shank_body, locationInChild, orientationInChild);

        PinJoint* knee = new PinJoint("knee",
                *shank_body, locationInParent2, orientationInChild,
                *thigh_body, locationInChild, orientationInChild);

        PinJoint* hip = new PinJoint("hip",
                *thigh_body, locationInParent3, orientationInChild,
                *HAT_body, locationInChild, orientationInChild);
        PlanarJoint *pelvisToGround = new PlanarJoint("PelvisToGround",
            ground, *HAT_body);

        // A planar joint has three coordinates:
        //     RotationZ, TranslationX, TranslationY
        // Set the properties of the coordinates that define the joint
        Coordinate& pelvis_rz =
            pelvisToGround->updCoordinate(PlanarJoint::Coord::RotationZ);
        pelvis_rz.setName("pelvis_rot");
        double pelvis_rz_range[2] = { -Pi, Pi };
        pelvis_rz.setRange(pelvis_rz_range);
        pelvis_rz.setDefaultValue(convertDegreesToRadians(0));
        //pelvis_rz.setDefaultLocked(true);

        Coordinate& pelvis_tx =
            pelvisToGround->updCoordinate(PlanarJoint::Coord::TranslationX);
        pelvis_tx.setName("pelvis_mov_x");
        double pelvis_tx_range[2] = { -10, 10 };
        pelvis_tx.setRange(pelvis_tx_range);
        pelvis_tx.setDefaultValue(0);

        Coordinate& pelvis_ty =
            pelvisToGround->updCoordinate(PlanarJoint::Coord::TranslationY);
        pelvis_ty.setName("pelvis_mov_y");
        double pelvis_ty_range[2] = { -1, 2 };
        pelvis_ty.setRange(pelvis_ty_range);
        pelvis_ty.setDefaultValue(0.843);

         osimModel.addJoint(pelvisToGround); 
    /*	double allStiff = 10000, allDamping = 5., allTransition = 10.;
    	auto toeLimitForce = new CoordinateLimitForce("q0", toeRange[1]*180/Pi,
        allStiff, toeRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	auto ankleLimitForce = new CoordinateLimitForce("q1", ankleRange[1]*180/Pi,
        allStiff, ankleRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	auto kneeLimitForce = new CoordinateLimitForce("q2", kneeRange[1]*180/Pi,
        allStiff, kneeRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	auto hipLimitForce = new CoordinateLimitForce("q3", hipRange[1]*180/Pi,
        allStiff, hipRange[0]*180/Pi, allStiff, allDamping, allTransition);
    	//tip->addComponent(toeLimitForce);
    	//ankle->addComponent(ankleLimitForce);
    	//knee->addComponent(kneeLimitForce);
    	//hip->addComponent(hipLimitForce);*/



        // Add the bodies to the model
        osimModel.addBody(foot_body);
        osimModel.addBody(shank_body);
        osimModel.addBody(thigh_body);
        osimModel.addBody(HAT_body);

        // Add the joints to the model
        //osimModel.addJoint(tip);
        osimModel.addJoint(ankle);
        osimModel.addJoint(knee);
        osimModel.addJoint(hip);

        //tip->updCoordinate().setName("q0");
        ankle->updCoordinate().setName("q1_rot");
        knee->updCoordinate().setName("q2_rot");
        hip->updCoordinate().setName("q3_rot");
	auto& coordSet = osimModel.updCoordinateSet();

        double cx[40];
        double cy[40];int sz;
readx_y("src/delp1.txt",cx,cy,sz);SimmSpline flc1(sz, cx,cy);double ankleRange[2]={cx[0],cx[sz-1]};
readx_y("src/delp2.txt",cx,cy,sz);SimmSpline flc2(sz, cx,cy);
readx_y("src/delp3.txt",cx,cy,sz);SimmSpline flc3(sz, cx,cy);
readx_y("src/delp4.txt",cx,cy,sz);SimmSpline flc4(sz, cx,cy);double kneeRange[2]={cx[0],cx[sz-1]};
readx_y("src/delp5.txt",cx,cy,sz);SimmSpline flc5(sz, cx,cy);double hipRange[2]={cx[0],cx[sz-1]};
readx_y("src/delp6.txt",cx,cy,sz);SimmSpline flc6(sz, cx,cy);
        auto* a1=new DelpActuator("ap",16.,1,.011,.068,0.,"src/delp1.txt",&coordSet.get("q1_rot"));
        auto* a2=new DelpActuator("kp",18.,1,.011,.068,0.,"src/delp4.txt",&coordSet.get("q2_rot"));
        auto* a3=new DelpActuator("hp",20.,1,.011,.068,0.,"src/delp5.txt",&coordSet.get("q3_rot"));
        auto* a_1=new DelpActuator("am",16.,-1,.011,.068,0.,"src/delp2.txt",&coordSet.get("q1_rot"));
        auto* a_2=new DelpActuator("km",18.,-1,.011,.068,0.,"src/delp3.txt",&coordSet.get("q2_rot"));
        auto* a_3=new DelpActuator("hm",20.,-1,.011,.068,0.,"src/delp6.txt",&coordSet.get("q3_rot"));
        a1->setTorqueAngleCurve(flc1);a2->setTorqueAngleCurve(flc4);a3->setTorqueAngleCurve(flc5);
        a_1->setTorqueAngleCurve(flc2);a_2->setTorqueAngleCurve(flc3);a_3->setTorqueAngleCurve(flc6);
        cout<<":::"<<flc1.getXValues()[0]<<endl;
        osimModel.addComponent(a1);
        osimModel.addComponent(a2);
        osimModel.addComponent(a3);
        osimModel.addComponent(a_1);
        osimModel.addComponent(a_2);
        osimModel.addComponent(a_3);
        
        // define the simulation times
    
    Actuator &kp=osimModel.updComponent<DelpActuator>("kp") ; 
    //cout<<osimModel.updForceSet().updActuators().getSize()<<endl;   
    //osimModel.updForceSet().updActuators().adoptAndAppend(&kp);   
    //cout<<osimModel.updForceSet().updActuators().getSize()<<endl;   
   // cout<<"mmmm:"<<osimModel.getActuators().getSize()<<endl;
    Array<string> delpNames;
    delpNames.append("ap");delpNames.append("kp");delpNames.append("hp");
    delpNames.append("am");delpNames.append("km");delpNames.append("hm");
    ActuatorPowerProbe* delpWorkProbe = new ActuatorPowerProbe(delpNames, false, 1);
    delpWorkProbe->setName("delpWork");
    delpWorkProbe->setOperation("integrate");
    SimTK::Vector ic1(6,0.);
    //ic1 = 0.0;      // some arbitrary initial condition.
    delpWorkProbe->setInitialConditions(ic1);
cout<<__LINE__<<endl;
    //osimModel.addProbe(delpWorkProbe);
    //cout<<"probe added"<<endl;

    Array<string> joint_names;
    osimModel.getJointSet().getNames(joint_names);

    JointInternalPowerProbe* jointWorkProbe = new JointInternalPowerProbe(joint_names, false, 1);
    jointWorkProbe->setName("JointWork");
    jointWorkProbe->setOperation("integrate");
    jointWorkProbe->setInitialConditions(SimTK::Vector(4, 0.0));
   // osimModel.addProbe(jointWorkProbe);


        double toeRange[2] = {0, Pi/2};
       // double ankleRange[2]={a1->getDelpLowAngle(),a1->getDelpHighAngle()};
       // double kneeRange[2] ={a2->getDelpLowAngle(),a2->getDelpHighAngle()} ;
       // double hipRange[2] ={a3->getDelpLowAngle(),a3->getDelpHighAngle()} ;
        cout<<ankleRange[0]*180/Pi<<","<<ankleRange[1]*180/Pi<<endl;
        cout<<kneeRange[0]*180/Pi<<","<<kneeRange[1]*180/Pi<<endl;
        cout<<hipRange[0]*180/Pi<<","<<hipRange[1]*180/Pi<<endl;

       double q0=80,q1=-100,q2=40,q3=-20 ;

        //tip->updCoordinate().setRange(toeRange);
        ankle->updCoordinate().setRange(ankleRange);
        ankle->updCoordinate().setDefaultValue(convertDegreesToRadians(q1));
        knee->updCoordinate().setRange(kneeRange);
        knee->updCoordinate().setDefaultValue(convertDegreesToRadians(q2));
        hip->updCoordinate().setRange(hipRange);
        hip->updCoordinate().setDefaultValue(convertDegreesToRadians(q3));
        // add the controller to the model
        PrescribedController* actcontroller =  new PrescribedController();
        actcontroller->addActuator(*a1);
        actcontroller->addActuator(*a2);
        actcontroller->addActuator(*a3);
        actcontroller->addActuator(*a_1);
        actcontroller->addActuator(*a_2);
        actcontroller->addActuator(*a_3);

        actcontroller->prescribeControlForActuator("ap", new Constant(0));
        actcontroller->prescribeControlForActuator("kp", new Constant(0));
        actcontroller->prescribeControlForActuator("hp", new Constant(0));
        actcontroller->prescribeControlForActuator("am", new Constant(0));
        actcontroller->prescribeControlForActuator("km", new Constant(0));
        actcontroller->prescribeControlForActuator("hm", new Constant(0));
        actcontroller->set_interpolation_method(3);
    
        osimModel.addController(actcontroller);

        //const ControllerSet &cs= osimModel.getControllerSet();
        osimModel.setGravity(Vec3(0., -9.81   , 0.));
        // debugging.
	
//start wrap spring
//      original length is 8 cm and stiffenes is 350N to each 8cm---for each spring
//      350N/0.08m=4375
	double pullymass=.001,stiffness=4454.76*4.,dissipation=0.01;
	double pullyrad=0.05,pullylength=0.05,pullyinertia=.0001;
           // body that acts as the pulley that the path wraps over

	WrapCylinder* pulley1 = new WrapCylinder();
    	pulley1->set_radius(pullyrad); pulley1->set_length(pullylength); pulley1->set_quadrant("-y");
	pulley1->setName("wrap1");
        OpenSim::Body* pulleyBody1 =
        new OpenSim::Body("PulleyBody1", pullymass ,Vec3(0),  pullymass*Inertia::sphere(.0001));
    	pulleyBody1->addWrapObject(pulley1);
    	osimModel.addBody(pulleyBody1);
        WeldJoint* weld1 =
        new WeldJoint("weld1", *thigh_body, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody1, Vec3(0), Vec3(0));
        osimModel.addJoint(weld1);

	WrapCylinder* pulley2 = new WrapCylinder();
    	pulley2->set_radius(pullyrad); pulley2->set_length(pullylength); pulley2->set_quadrant("-x");
	pulley2->setName("wrap2");
        OpenSim::Body* pulleyBody2 =
        new OpenSim::Body("PulleyBody2", pullymass ,Vec3(0),  pullymass*Inertia::sphere(.0001));
    	pulleyBody2->addWrapObject(pulley2);
    	osimModel.addBody(pulleyBody2);
        WeldJoint* weld2 =
        new WeldJoint("weld2", *HAT_body, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody2, Vec3(0), Vec3(0));
        osimModel.addJoint(weld2);

	WrapCylinder* pulley3 = new WrapCylinder();
    	pulley3->set_radius(pullyrad); pulley3->set_length(pullylength); pulley3->set_quadrant("-x");
	pulley3->setName("wrap3");
        OpenSim::Body* pulleyBody3 =
        new OpenSim::Body("PulleyBody3", pullymass ,Vec3(0),  pullymass*Inertia::sphere(.0001));
    	pulleyBody3->addWrapObject(pulley3);
    	osimModel.addBody(pulleyBody3);
        WeldJoint* weld3 =
        new WeldJoint("weld3", *shank_body, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody3, Vec3(0), Vec3(0));
        osimModel.addJoint(weld3);

	double resting_length=0.25;
    	PathSpring* spring1 =
        new PathSpring("knee_spring",.4,stiffness ,dissipation);
    	spring1->updGeometryPath().
        appendNewPathPoint("origin1", *thigh_body, Vec3(pullyrad, 0.2, 0));
    	spring1->updGeometryPath().
        appendNewPathPoint("insert1", *shank_body, Vec3(pullyrad,shankLength-.2,0));
    	spring1->updGeometryPath().addPathWrap(*pulley1);
    	PathSpring* spring2 =
        new PathSpring("hip_spring",resting_length,stiffness ,dissipation);
    	spring2->updGeometryPath().
        appendNewPathPoint("origin2", *HAT_body, Vec3(-pullyrad, 0.2, 0));
    	spring2->updGeometryPath().
        appendNewPathPoint("insert2", *thigh_body, Vec3(-pullyrad,thighLength-.05,0));
    	spring2->updGeometryPath().addPathWrap(*pulley2);
    	PathSpring* spring3 =
        new PathSpring("ankle_spring",resting_length,stiffness ,dissipation);
    	spring3->updGeometryPath().
        appendNewPathPoint("origin3", *shank_body, Vec3(-pullyrad, 0.2, 0));
    	spring3->updGeometryPath().
        appendNewPathPoint("insert3", *foot_body, Vec3(-pullyrad,footLength-.05,0));
    	spring3->updGeometryPath().addPathWrap(*pulley3);
cout<<__LINE__<<endl;
    	osimModel.addForce(spring1);
    	osimModel.addForce(spring2);
    	osimModel.addForce(spring3);
cout<<__LINE__<<endl;
//create contact between low point and ground
    	ContactHalfSpace *floor =
        new ContactHalfSpace(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI), ground, "floor");
    	ContactSphere *tipc= new ContactSphere(0.02,Vec3(0),*foot_body,"tipcontact");
    	ContactSphere *tipa= new ContactSphere(0.02,Vec3(0),*shank_body,"ankcontact");
    	osimModel.addComponent(floor);
    	osimModel.addComponent(tipc);
    	osimModel.addComponent(tipa);
	double cstiffness = 60677760, cdissipation = 2, friction = 0.8, viscosity = 0.5;
 	OpenSim::SmoothSphereHalfSpaceForce* ftip=new OpenSim::SmoothSphereHalfSpaceForce("tforce",
			*tipc,*floor);
        ftip->set_stiffness(cstiffness);ftip->set_dissipation(cdissipation);
        ftip->set_static_friction(friction);ftip->set_dynamic_friction(friction);
        ftip->set_viscous_friction(0.5);ftip->set_transition_velocity(0.2); 
	//ftip->set_derivative_smoothing(1e-5);
	ftip->set_hertz_smoothing(300);
	ftip->set_hunt_crossley_smoothing(50);
 	OpenSim::SmoothSphereHalfSpaceForce* atip=new OpenSim::SmoothSphereHalfSpaceForce("aforce",
			*tipa,*floor);
        atip->set_stiffness(cstiffness);atip->set_dissipation(cdissipation);
        atip->set_static_friction(friction);atip->set_dynamic_friction(friction);
        atip->set_viscous_friction(0.5);atip->set_transition_velocity(0.2); 
	//ftip->set_derivative_smoothing(1e-5);
	atip->set_hertz_smoothing(300);
	ftip->set_hunt_crossley_smoothing(50);
        osimModel.addComponent(ftip);
        osimModel.addComponent(atip);

cout<<__LINE__<<endl;
    osimModel.finalizeConnections();
cout<<__LINE__<<endl;
 
    cout<<"actsize:"<<osimModel.upd_ForceSet().updActuators().getSize()<<endl;   
cout<<__LINE__<<endl;

        // Initialize system
        osimModel.buildSystem();
cout<<__LINE__<<endl;
        //default activation must come before initstate

        State &si = osimModel.initializeState();
cout<<__LINE__<<endl;

        // Pin joint initial states
        CoordinateSet &coordinates = osimModel.updCoordinateSet();
        coordinates[3].setValue(si,q1*Pi/180, true);
        coordinates[4].setValue(si,q2*Pi/180, true);
        coordinates[5].setValue(si,q3*Pi/180, true);
        //for (const auto& coord :  osimModel.updCoordinateSet()) cout<<coord.getName()<<endl;
        // Setup ForceReporter and Manager
       // ForceReporter* forces = new ForceReporter(&osimModel);  
       // osimModel.updAnalysisSet().adoptAndAppend(forces);
cout<<__LINE__<<endl;
//
//find initial optimal values for initial state(after setting angles)
	//Vector udot(4,0.);
        //InverseDynamicsSolver insol(osimModel);
        //Vector init_tou=insol.solve(si,udot);
        //cout<<"torques for static equilibrium:"<<init_tou<<endl;
        osimModel.printDetailedInfo(si, std::cout);
        //

        si.getQ().dump("Initial q's");
        si.getU().dump("Initial u's");
        si.setTime(0);
cout<<__LINE__<<endl;


        
        cout<<"___________\nSpringdata:"<<endl;
        osimModel.getMultibodySystem().realize(si, Stage::Position);
        osimModel.getMultibodySystem().realize(si, Stage::Velocity);
        cout<<"spring length:"<<spring1->getLength(si)<<"\t"<<spring2->getLength(si)
	<<"\t"<<spring3->getLength(si)<<endl;
        cout<<"spring stretch:"<<spring1->getStretch(si)<<"\t"<<spring2->getStretch(si)
	<<"\t"<<spring3->getStretch(si)<<endl;
        cout<<"spring tension:"<<spring1->getTension(si)<<"\t"<<spring2->getTension(si)
	<<"\t"<<spring3->getTension(si)<<endl;
        //cout<<"spring arm:"<<spring1->computeMomentArm(si, coordinates[2])
        
        osimModel.print("w3springs.osim");
cout<<__LINE__<<endl;
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception in 4links_example: " << ex.what() << std::endl;
        return 1;
    }

    cout << "Done." << endl;
    return 0;
}
