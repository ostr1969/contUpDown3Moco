
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
#include "readx_y.h"
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
	ofstream Arms("src/ellipseArms.csv", ofstream::out);
        Model osimModel;
        osimModel.setName("ellipse3springCont");
        osimModel.setAuthors("barak ostraich");

            
        // Get the ground body.
        Ground& ground = osimModel.updGround();
        ground.attachGeometry(new Mesh("checkered_floor.vtp"));

        // Create first linkage body.
        double linkageMass1 = 2.32 , linkageLength1 = 0.2084, linkageDiameter = 0.02;
        double linkageMass2 = 7.44 , linkageLength2 = 0.4182;
        double linkageMass3 = 16   , linkageLength3 = 0.4165;
        double linkageMass4 = 54.24, linkageLength4 = 0.7780;
        
        Vec3 linkageMassCenter1(0.,linkageLength1-0.1140836,0.);
        Vec3 linkageMassCenter2(0.,linkageLength2-0.18647538,0.);
        Vec3 linkageMassCenter3(0.,linkageLength3-0.17055675,0.);
        Vec3 linkageMassCenter4(0.,linkageLength4-0.4237,0.);
        double exothick=0.015;
        Vec3 exocenter(0.,exothick/2,0.);

        OpenSim::Body* linkage1 = new OpenSim::Body("foot", linkageMass1,
                linkageMassCenter1, Inertia(0.1,0.1,0.00662,0.,0.,0.));
        OpenSim::Body* linkage2 = new OpenSim::Body("shank", linkageMass2,
                linkageMassCenter2, Inertia(0.1,0.1,0.1057,0.,0.,0.));
        OpenSim::Body* linkage3 = new OpenSim::Body("thigh", linkageMass3,
                linkageMassCenter3, Inertia(0.2,0.1,0.217584,0.,0.,0.));
        OpenSim::Body* linkage4 = new OpenSim::Body("HAT", linkageMass4,
                linkageMassCenter4, Inertia(1.1,1.1,1.48,0.,0.,0.));

        //create z vector for all joints 
        Rotation R = Rotation(-Pi/2, ZAxis); 
        // Graphical representation of foot.
        Sphere sphere(0.02);
	Ellipsoid head(.15/2,.25/2,.15/2);
        //Sphere psphere(0.01);
        //psphere.upd_Appearance().set_color(SimTK::Vec3(1.0, 0.0, 0.0));
        Cylinder exo(.05, exothick);
	Brick brick(Vec3(.05/2, 0.01, .01));
	
        Cylinder cyl1(linkageDiameter/2, linkageLength1/2);
        Frame* cyl1Frame = new PhysicalOffsetFrame(*linkage1, 
            Transform(Vec3(0.0, linkageLength1/2 , 0.0)));
        cyl1Frame->setName("Cyl1_frame");
        cyl1Frame->attachGeometry(cyl1.clone());
        //cyl1Frame->attachGeometry(exo.clone());
        // cyl1Frame->attachGeometry(new Mesh("foot.vtp"));
        Frame* b1Frame = new PhysicalOffsetFrame(*linkage1,
            Transform(Vec3(-0.05/2, linkageLength1-.05, 0.)));
	b1Frame->attachGeometry(brick.clone());
        osimModel.addComponent(b1Frame);
        osimModel.addComponent(cyl1Frame);

        // Create shank body.
        Cylinder cyl2(linkageDiameter/2, linkageLength2/2);
        Frame* cyl2Frame = new PhysicalOffsetFrame(*linkage2,
            Transform(Vec3(0.0, linkageLength2/2 , 0.0)));
        cyl2Frame->setName("Cyl2_frame");
        cyl2Frame->attachGeometry(cyl2.clone());
        cyl2Frame->attachGeometry(exo.clone());

        Frame* b2Frame = new PhysicalOffsetFrame(*linkage2,
            Transform(Vec3(0.05/2, linkageLength2-.05, 0.)));
	b2Frame->attachGeometry(brick.clone());
        osimModel.addComponent(b2Frame);
        osimModel.addComponent(cyl2Frame);
        // Create thigh body.
        linkage3->attachGeometry(sphere.clone());
        Cylinder cyl3(linkageDiameter/2, linkageLength3/2);
        Frame* cyl3Frame = new PhysicalOffsetFrame(*linkage3,
            Transform(Vec3(0.0, linkageLength3/2 , 0.0)));
        cyl3Frame->setName("Cyl3_frame");
        cyl3Frame->attachGeometry(cyl3.clone());
        cyl3Frame->attachGeometry(exo.clone());
        //cyl3Frame->attachGeometry(new Mesh("femur_r.vtp"));
        Frame* b3Frame = new PhysicalOffsetFrame(*linkage3,
            Transform(Vec3(-0.05/2, linkageLength3-.05, 0.)));
	b3Frame->attachGeometry(brick.clone());
        osimModel.addComponent(b3Frame);
        osimModel.addComponent(cyl3Frame);
        // Create HAT body.
        linkage4->attachGeometry(sphere.clone());
        Cylinder cyl4(linkageDiameter/2, linkageLength4/2);
        Frame* cyl4Frame = new PhysicalOffsetFrame(*linkage4,
            Transform(Vec3(0.0, linkageLength4 / 2.0, 0.0)));
        Frame* exo4Frame = new PhysicalOffsetFrame(*linkage4,
            Transform(Vec3(0.0, 0.2, 0.0)));
        Frame* headFrame = new PhysicalOffsetFrame(*linkage4,
            Transform(Vec3(0.0, linkageLength4, 0.0)));
	headFrame->attachGeometry(head.clone());
        cyl4Frame->setName("Cyl4_frame");
        cyl4Frame->attachGeometry(cyl4.clone());
        exo4Frame->attachGeometry(exo.clone());
        osimModel.addComponent(cyl4Frame);
        osimModel.addComponent(headFrame);
        osimModel.addComponent(exo4Frame);
//create markers

    	Marker*m0 = new Marker();
    	m0->setName("m0");
    	m0->setParentFrame(*linkage4);
    	m0->set_location(SimTK::Vec3(-0.05,0,0));
     	osimModel.addMarker(m0);
    	Marker*m1 = new Marker();
    	m1->setName("m1");
    	m1->setParentFrame(*linkage3);
    	m1->set_location(SimTK::Vec3(-0.05,linkageLength3,0));
     	osimModel.addMarker(m1);
    	Marker*m2 = new Marker();
    	m2->setName("m2");
    	m2->setParentFrame(*linkage3);
    	m2->set_location(SimTK::Vec3(0.05,0,0));
     	osimModel.addMarker(m2);
    	Marker*m3 = new Marker();
    	m3->setName("m3");
    	m3->setParentFrame(*linkage2);
    	m3->set_location(SimTK::Vec3(0.05,linkageLength2,0));
     	osimModel.addMarker(m3);
    	Marker*m4 = new Marker();
    	m4->setName("m4");
    	m4->setParentFrame(*linkage2);
    	m4->set_location(SimTK::Vec3(-0.05,0,0));
     	osimModel.addMarker(m4);
    	Marker*m5 = new Marker();
    	m5->setName("m5");
    	m5->setParentFrame(*linkage1);
    	m5->set_location(SimTK::Vec3(-0.05,linkageLength1,0));
     	osimModel.addMarker(m5);
        Marker *tipmarker=new Marker("tipm",*linkage1,Vec3(0));
	osimModel.addMarker(tipmarker);


        // Create 1 degree-of-freedom pin joints between the bodies to create a
        // kinematic chain from ground through the block.
        Vec3 orientationInGround(0.);
        Vec3 locationInGround(0.);
        Vec3 locationInParent1(0.0, linkageLength1, 0.0);
        Vec3 locationInParent2(0.0, linkageLength2, 0.0);
        Vec3 locationInParent3(0.0, linkageLength3, 0.0);
        Vec3 locationInParent4(0.0, linkageLength4, 0.0);
        Vec3 orientationInChild(0.);
        Vec3 locationInChild(0.);

        PinJoint* tip   = new PinJoint("tip",
                ground, locationInGround, orientationInGround,
                *linkage1, locationInChild, orientationInChild);

        PinJoint* ankle = new PinJoint("ankle",
                *linkage1, locationInParent1, orientationInChild,
                *linkage2, locationInChild, orientationInChild);

        PinJoint* knee = new PinJoint("knee",
                *linkage2, locationInParent2, orientationInChild,
                *linkage3, locationInChild, orientationInChild);

        PinJoint* hip = new PinJoint("hip",
                *linkage3, locationInParent3, orientationInChild,
                *linkage4, locationInChild, orientationInChild);
        PlanarJoint *pelvisToGround = new PlanarJoint("PelvisToGround",
            ground, *linkage4);

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
        osimModel.addBody(linkage1);
        osimModel.addBody(linkage2);
        osimModel.addBody(linkage3);
        osimModel.addBody(linkage4);

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
	double pullyrad=0.05,pullylength=0.05;
           // body that acts as the pulley that the path wraps over

  /*  WrapCylinder* pulley1_ = new WrapCylinder();
    pulley1_->set_radius(pullyrad/2); pulley1_->set_length(pullylength); pulley1_->set_quadrant("-y");
    pulley1_->setName("wrap1_");pulley1_->set_translation(Vec3(0,-.05,0));
    WrapCylinder* pulley_ = new WrapCylinder();
    pulley_->set_radius(pullyrad/2); pulley_->set_length(pullylength); pulley_->set_quadrant("+x");
    pulley_->setName("wrap1_");pulley_->set_translation(Vec3(0,-.05,0));*/
        //OpenSim::Body* pulleyBody1 =
        //new OpenSim::Body("PulleyBody1", pullymass ,Vec3(0),  pullymass*Inertia::sphere(0.1));
    	//pulleyBody1->addWrapObject(pulley1);
    	//pulleyBody1->addWrapObject(pulley1_);
    	//pulleyBody1->addWrapObject(pulley_);
   // 	osimModel.addBody(pulleyBody1);
        //WeldJoint* weld1 =
        //new WeldJoint("weld1", *linkage3, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody1, Vec3(0), Vec3(0));
   // WeldJoint* weld1_ =
   // new WeldJoint("weld1_", *linkage3, Vec3(0, -0.05, 0), Vec3(0), *pulleyBody1_, Vec3(0), Vec3(0));
   //     osimModel.addJoint(weld1);
   //     osimModel.addJoint(weld1_);

	WrapEllipsoid* ellip = new WrapEllipsoid();
	ellip->set_dimensions(Vec3(.05,.08,.1));ellip->set_quadrant("-y");
	ellip->setName("wrap1");

	WrapCylinder* pulley1 = new WrapCylinder();
    	pulley1->set_radius(pullyrad); pulley1->set_length(pullylength); pulley1->set_quadrant("-y");
	pulley1->setName("wrap1");
        OpenSim::Body* pulleyBody1 =
        new OpenSim::Body("PulleyBody1", pullymass ,Vec3(0),  pullymass*Inertia::sphere(0.1));
	pulleyBody1->addWrapObject(ellip);
    	osimModel.addBody(pulleyBody1);
        WeldJoint* weld1 =
        new WeldJoint("weld1", *linkage3, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody1, Vec3(0), Vec3(0));
        osimModel.addJoint(weld1);

	WrapCylinder* pulley2 = new WrapCylinder();
    	pulley2->set_radius(pullyrad); pulley2->set_length(pullylength); pulley2->set_quadrant("-x");
	pulley2->setName("wrap2");
        OpenSim::Body* pulleyBody2 =
        new OpenSim::Body("PulleyBody2", pullymass ,Vec3(0),  pullymass*Inertia::sphere(0.1));
    	pulleyBody2->addWrapObject(pulley2);
    	osimModel.addBody(pulleyBody2);
        WeldJoint* weld2 =
        new WeldJoint("weld2", *linkage4, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody2, Vec3(0), Vec3(0));
        osimModel.addJoint(weld2);

	WrapCylinder* pulley3 = new WrapCylinder();
    	pulley3->set_radius(pullyrad); pulley3->set_length(pullylength); pulley3->set_quadrant("-x");
	pulley3->setName("wrap3");
        OpenSim::Body* pulleyBody3 =
        new OpenSim::Body("PulleyBody3", pullymass ,Vec3(0),  pullymass*Inertia::sphere(0.1));
    	pulleyBody3->addWrapObject(pulley3);
    	osimModel.addBody(pulleyBody3);
        WeldJoint* weld3 =
        new WeldJoint("weld3", *linkage2, Vec3(0, 0.0, 0), Vec3(0), *pulleyBody3, Vec3(0), Vec3(0));
        osimModel.addJoint(weld3);

	// Add the wrap object to the body, which takes ownership of it

	double resting_length=0.25;

    	PathSpring* spring1 =
        new PathSpring("knee_spring",0.25,stiffness ,dissipation);
    	spring1->updGeometryPath().
        appendNewPathPoint("origin1", *linkage3, Vec3(pullyrad, 0.05, 0));
    	spring1->updGeometryPath().
        //appendNewPathPoint("origin2", *linkage3, Vec3(pullyrad, 0.0, 0));
    	//spring1->updGeometryPath().
        appendNewPathPoint("insert1", *linkage2, Vec3(pullyrad,linkageLength2-.2,0));
    	spring1->updGeometryPath().addPathWrap(*ellip);
    	//spring1->updGeometryPath().addPathWrap(*pulley1_);
    	//spring1->updGeometryPath().addPathWrap(*pulley_);

    	PathSpring* spring2 =
        new PathSpring("hip_spring",resting_length,stiffness ,dissipation);
    	spring2->updGeometryPath().
        appendNewPathPoint("origin2", *linkage4, Vec3(-pullyrad, 0.2, 0));
    	spring2->updGeometryPath().
        appendNewPathPoint("insert2", *linkage3, Vec3(-pullyrad,linkageLength3-.05,0));
    	spring2->updGeometryPath().addPathWrap(*pulley2);
    	PathSpring* spring3 =
        new PathSpring("ankle_spring",resting_length,stiffness ,dissipation);
    	spring3->updGeometryPath().
        appendNewPathPoint("origin3", *linkage2, Vec3(-pullyrad, 0.2, 0));
    	spring3->updGeometryPath().
        appendNewPathPoint("insert3", *linkage1, Vec3(-pullyrad,linkageLength1-.05,0));
    	spring3->updGeometryPath().addPathWrap(*pulley3);
cout<<__LINE__<<endl;
    	osimModel.addForce(spring1);
    	osimModel.addForce(spring2);
    	osimModel.addForce(spring3);
cout<<__LINE__<<endl;
//create contact between low point and ground
    	ContactHalfSpace *floor =
        new ContactHalfSpace(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI), ground, "floor");
    	ContactSphere *tipc= new ContactSphere(0.02,Vec3(0),*linkage1,"tipcontact");
    	ContactSphere *tipa= new ContactSphere(0.02,Vec3(0),*linkage2,"ankcontact");
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

    osimModel.finalizeConnections();
 
    cout<<"actsize:"<<osimModel.upd_ForceSet().updActuators().getSize()<<endl;   

        // Initialize system
        osimModel.buildSystem();
cout<<__LINE__<<endl;
        //default activation must come before initstate

        State &si = osimModel.initializeState();
cout<<__LINE__<<endl;

        // Pin joint initial states
        CoordinateSet &coordinates = osimModel.updCoordinateSet();
       // coordinates[0].setValue(si, 0, true);
       // coordinates[1].setValue(si,0, true);
       // coordinates[2].setValue(si, , true);
        coordinates[3].setValue(si,q1*Pi/180, true);
        coordinates[4].setValue(si,q2*Pi/180, true);
        coordinates[5].setValue(si,q3*Pi/180, true);
        //for (const auto& coord :  osimModel.updCoordinateSet()) cout<<coord.getName()<<endl;
        // Setup ForceReporter and Manager
       // ForceReporter* forces = new ForceReporter(&osimModel);  
       // osimModel.updAnalysisSet().adoptAndAppend(forces);
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
	cout<<"ARMS\n";
       for( int r=(int)(kneeRange[0]*180/Pi); r<(int)(kneeRange[1]*180/Pi);r++){
       coordinates[4].setValue(si,r*Pi/180, true);
       Arms<<r<<","<<spring1->computeMomentArm(si, coordinates[4])<<endl;}
       Arms.close();
        
        osimModel.print("ellipse.osim");
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
