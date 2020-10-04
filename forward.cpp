//#include <Moco/Components/DelpActuator.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Analyses/ProbeReporter.h>
#include <OpenSim/Analyses/ForceReporter.h>
//#include <OpenSim/Actuators/DelpActuator.h>
#include "DelpActuator.h"
#include <OpenSim/Common/STOFileAdapter.h>
#include <Moco/osimMoco.h>
#define MYFWD 1
#include "additions.h"
#include "myActuatorPowerProbe.cpp"
#include "console.h"
using namespace OpenSim;
using namespace SimTK;
using namespace std;
ofstream debugLog("results/fwd_debug.csv", ofstream::out);
ofstream Log1("results/fwd_1Table.csv", ofstream::out);
ofstream Log2("results/fwd_2Table.csv", ofstream::out);
ofstream Log3("results/fwd_3Table.csv", ofstream::out);
ofstream Log4("results/fwd_4Table.csv", ofstream::out);
ofstream Log5("results/fwd_5Table.csv", ofstream::out);
ofstream Log6("results/fwd_6Table.csv", ofstream::out);
ofstream LogA("results/fwd_AllTable.csv", ofstream::out);


class MyReporter : public PeriodicEventReporter {
public:
    MyReporter(const MultibodySystem& system, Real interval , Model& model)
            : PeriodicEventReporter(interval), system(system), _model(model) {}

    // Show x-y position of the pendulum weight as a function of time.
    void handleEvent(const State& state) const override {
        Vector_<SpatialVec>  forcesAtMInG;
        system.realize(state, Stage::Acceleration);
        //system.getMatterSubsystem().calcMobilizerReactionForces( state,forcesAtMInG);
        SimTK::Vec3 comPos =
                system.getMatterSubsystem().calcSystemMassCenterLocationInGround(state);
        Vec3 COM_v = system.getMatterSubsystem().calcSystemMassCenterVelocityInGround(state);
	double totEne=system.calcEnergy(state);

        auto& cont1 = _model.getComponent<OpenSim::SmoothSphereHalfSpaceForce>(
                  "tforce");
        auto& fset=_model.getForceSet();
        auto& controller=_model.getControllerSet()[0];
        auto& tip=_model.getMarkerSet()[6];
	//cout<<tip.getLocationInGround(state)<<endl;


        Vector q=state.getQ();
        Vector u=state.getU();
        const Set<const Actuator>& ASet = controller.getActuatorSet();
        const DelpActuator* act1=dynamic_cast<const DelpActuator*>(&ASet[0]);
        const DelpActuator* act2=dynamic_cast<const DelpActuator*>(&ASet[1]);
        const DelpActuator* act3=dynamic_cast<const DelpActuator*>(&ASet[2]);
        const DelpActuator* act4=dynamic_cast<const DelpActuator*>(&ASet[3]);
        const DelpActuator* act5=dynamic_cast<const DelpActuator*>(&ASet[4]);
        const DelpActuator* act6=dynamic_cast<const DelpActuator*>(&ASet[5]);
	double P1=act1->getPower(state);
	double P2=act2->getPower(state);
	double P3=act3->getPower(state);
	double P4=act4->getPower(state);
	double P5=act5->getPower(state);
	double P6=act6->getPower(state);
	 PathSpring* spK = dynamic_cast<PathSpring*>(&fset.get(0));
	 PathSpring* spH = dynamic_cast<PathSpring*>(&fset.get(1));
	 PathSpring* spA = dynamic_cast<PathSpring*>(&fset.get(2));
        double springTK=spK->getTension(state)*0.05;//torque knee
	double spPK=spK->getLengtheningSpeed(state)*spK->getTension(state);//w*tou=Power
	double spEK=0.5*spK->getStretch(state)*spK->getStretch(state)*
			spK->getStiffness();
        //double spEK2=spK->computePotentialEnergy(state);
        double springTH=spH->getTension(state)*0.05;//hip
	double spPH=spH->getLengtheningSpeed(state)*spH->getTension(state);
	double spEH=0.5*spH->getStretch(state)*spH->getStretch(state)*
			spH->getStiffness();
        double springTA=spA->getTension(state)*0.05;//ankle
	double spPA=spA->getLengtheningSpeed(state)*spA->getTension(state);
	double spEA=0.5*spA->getStretch(state)*spA->getStretch(state)*
			spA->getStiffness();

        Log1<< state.getTime()<<","<<q(1)<<","<<u(1)<<","<<act1->getControl(state)<<
        ","<<act1->getDelpVars(state,0)<<","<<act1->getDelpVars(state,1)<<","
        <<act1->getDelpVars(state,2)<<","<<act1->getDelpVars(state,3)
        <<","<<P1<<","<<act1->get_coordinate()<<","<<springTA<<","<<spPA<<endl;
        Log2<< state.getTime()<<","<<q(2)<<","<<u(2)<<","<<act2->getControl(state)<<
        ","<<act2->getDelpVars(state,0)<<","<<act2->getDelpVars(state,1)<<","
        <<act2->getDelpVars(state,2)<<","<<act2->getDelpVars(state,3)
        <<","<<P2<<","<<act2->get_coordinate()<<","<<springTK<<","<<spPK<<endl;
        Log3<< state.getTime()<<","<<q(3)<<","<<u(3)<<","<<act3->getControl(state)<<
        ","<<act3->getDelpVars(state,0)<<","<<act3->getDelpVars(state,1)<<","
        <<act3->getDelpVars(state,2)<<","<<act3->getDelpVars(state,3)
        <<","<<P3<<","<<act3->get_coordinate()<<","<<springTH<<","<<spPH<<endl;
        Log4<< state.getTime()<<","<<q(1)<<","<<u(1)<<","<<act4->getControl(state)<<
        ","<<act4->getDelpVars(state,0)<<","<<act4->getDelpVars(state,1)<<","
        <<act4->getDelpVars(state,2)<<","<<act4->getDelpVars(state,3)
        <<","<<P4<<","<<act4->get_coordinate()<<endl;
        Log5<< state.getTime()<<","<<q(2)<<","<<u(2)<<","<<act5->getControl(state)<<
        ","<<act5->getDelpVars(state,0)<<","<<act5->getDelpVars(state,1)<<","
        <<act5->getDelpVars(state,2)<<","<<act5->getDelpVars(state,3)
        <<","<<P5<<","<<act5->get_coordinate()<<endl;
        Log6<< state.getTime()<<","<<q(3)<<","<<u(3)<<","<<act6->getControl(state)<<
        ","<<act6->getDelpVars(state,0)<<","<<act6->getDelpVars(state,1)<<","
        <<act6->getDelpVars(state,2)<<","<<act6->getDelpVars(state,3)
        <<","<<P6<<","<<act6->get_coordinate()<<endl;
	double T1=act1->getDelpVars(state,0)+act4->getDelpVars(state,0);
	double T2=act2->getDelpVars(state,0)+act5->getDelpVars(state,0);
	double T3=act3->getDelpVars(state,0)+act6->getDelpVars(state,0);
	 /*CoordinateLimitForce* limf1 = dynamic_cast<CoordinateLimitForce*>(&_fset.get(1));
	 CoordinateLimitForce* limf2 = dynamic_cast<CoordinateLimitForce*>(&_fset.get(2));
	 CoordinateLimitForce* limf3 = dynamic_cast<CoordinateLimitForce*>(&_fset.get(3));
	 CoordinateLimitForce* limf4 = dynamic_cast<CoordinateLimitForce*>(&_fset.get(4));
         double limPow1=limf1->getPowerDissipation(state);
         double limPow2=limf2->getPowerDissipation(state);
         double limPow3=limf3->getPowerDissipation(state);
         double limPow4=limf4->getPowerDissipation(state);*/
	//double spE=0.5*sp->getStiffness()*sp->getStretch(state)*sp->getStretch(state);

        LogA<<state.getTime()<<","<<
	      q(0)<<","<<q(1)<<","<<q(2)<<","<<q(3)<<","<<
              u(0)<<","<<u(1)<<","<<u(2)<<","<<u(3)<<","<<
	      T1<<","<<T2<<","<<T3<<","<<springTA<<","<<springTK<<","<<
	      springTH<<","<<
	      P1+P4<<","<<P2+P5<<","<<P3+P6<<","<<
		spPA<<","<<spPK<<","<<spPH<<","<<
		spEA<<","<<spEK<<","<<spEH<<","<<
	//	limPow1<<","<<limPow2<<","<<limPow3<<","<<limPow4<<","<<
		comPos(1)<<","<<COM_v(0)<<","<<COM_v(1)<<
		","<<totEne<<","<<cont1.getRecordValues(state)[0]<<
		","<<cont1.getRecordValues(state)[1]<<endl;	      
//Array<double> f1=getModel().getComponent<Force>("tforce").getRecordValues(input.state);
//Array<double> f2=getModel().getComponent<Force>("aforce").getRecordValues(input.state);
        std::cout << state.getTime() <<"f:"<<cont1.getRecordValues(state)[0]<<   
		","<<cont1.getRecordValues(state)[1]<<",y:"<< comPos[1]<<",tip:"
		<<tip.getLocationInGround(state)<<endl; 
        
        	
	//	limf1->computePotentialEnergy(state)<<
	//	","<< limf2->computePotentialEnergy(state)<<","
	//	<< limf3->computePotentialEnergy(state)<<","
	//	<<limf4->computePotentialEnergy(state)<<std::endl;

    }

private:
    const MultibodySystem& system;
    const Model& _model;

};
ofstream fwddebugLog("results/fwd_debug.csv", ofstream::out);


//
int main(int argc, char *argv[]){
	InpVars data=readvars();
    Log1<<"t,ang,vel,E,tou,act,fac,opt,power,coord,spT,spP\n";
    Log2<<"t,ang,vel,E,tou,act,fac,opt,power,coord,spT,spP\n";
    Log3<<"t,ang,vel,E,tou,act,fac,opt,power,coord,spT,spP\n";
    Log4<<"t,ang,vel,E,tou,act,fac,opt,power,coord\n";
    Log5<<"t,ang,vel,E,tou,act,fac,opt,power,coord\n";
    Log6<<"t,ang,vel,E,tou,act,fac,opt,power,coord\n";
    LogA<<"t,ang1,ang2,ang3,ang4,v1,v2,v3,v4,TA,TK,TH,TspA,TspK,TspH,PA,PK,PH,spPA,spPK,spPH,spEA,spEK,spEH,y,vx,vy,totE,fx,fy\n";
    //LogA<<"t,ang1,ang2,ang3,ang4,v1,v2,v3,v4,T2,T3,T4,Tsp,P2,P3,P4,spP,limP1,limP2,limP3,limP4,y,vy\n";

	for (int i=0;i<2;i++) 
		cout<<data.ints[i].label<<":"<<data.ints[i].val<<endl;	
	for (int i=0;i<data.strings.size();i++) 
		cout<<data.strings[i].label<<":"<<data.strings[i].val<<endl;	
	for (int i=0;i<data.doubles.size();i++) 
		cout<<data.doubles[i].label<<":"<<data.doubles[i].val<<endl;	
	string modelFile="con3springs.osim";
	string mocofile="results/mycolo_traj.sto";
        TimeSeriesTable mocoTable(mocofile);
        //RowVectorView row0=mocoTable.getRowAtIndex(0);
        //int len=row0.size();
	//cout<<"moco springs(KHA):"<<row0(len-3)<<","<<row0(len-2)<<","<<row0(len-1)<<endl;
        string controlsfile="mycolo_controls.sto";
        string statesFile="results/mycolo_states.bin";
	string controlsFile="results/mycolo_controls.bin";
        myTrajectory coloControls=binToTraj(controlsFile);
	cout<<"read conrols"<<endl;
        myTrajectory coloStates=binToTraj(statesFile);
	cout<<"read initial state"<<endl;
        Vector st=coloStates.getFirstRow();
	cout<<st<<endl;
        Model osimModel(modelFile);
        osimModel.updControllerSet().remove(0);
        osimModel.initSystem();
    OpenSim::Array<std::string> actuNames;
    const auto modelPath = osimModel.getAbsolutePath();
    for (const auto& actu : osimModel.getComponentList<DelpActuator>()) {
        actuNames.append(actu.getAbsolutePathString());
    }
        updateDelpActuator(osimModel, actuNames[0],"src/delp1.txt",.011,.068, 1,16);
        updateDelpActuator(osimModel, actuNames[1],"src/delp4.txt",.011,.068, 1,18);
        updateDelpActuator(osimModel, actuNames[2],"src/delp5.txt",.011,.068, 1,20);
        updateDelpActuator(osimModel, actuNames[3],"src/delp2.txt",.011,.068,-1,16);
        updateDelpActuator(osimModel, actuNames[4],"src/delp3.txt",.011,.068,-1,18);
        updateDelpActuator(osimModel, actuNames[5],"src/delp6.txt",.011,.068,-1,20);

    const SimTK::Vector& time = coloControls.time;
    //std::vector<double> col{0, 1.};
    //Vector timv{2, 0.};timv(1)=0.45;
   // std::vector<double> col2{1., 1.};
    //Vector actv{2,0.0};

    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");

    for (int i = 0; i < actuNames.size(); ++i) {
        const auto control = coloControls.getControl(actuNames[i]);
//        auto* fun = new GCVSpline(5, time.nrow(), &time[0], &control[0]);
        auto* funL=new PiecewiseLinearFunction(time.nrow(), &time[0], &control[0]);

        const auto& actu = osimModel.getComponent<DelpActuator>(actuNames[i]);
        controller->addActuator(actu);
        controller->prescribeControlForActuator( actu.getName(), funL);
 	//OpenSim::Constant f(0.3);
        //controller->prescribeControlForActuator( actu.getName(), f);
        fwddebugLog<<"time,"<<actuNames[i]<<endl;
        for (int j=0;j<time.nrow();j++)
        fwddebugLog<<time[j]<<","<<control[j]<<endl;
    }
    
    osimModel.addController(controller);
    osimModel.updCoordinateSet()[2].setDefaultValue(0.86);
	
        State &si = osimModel.initSystem();
// 1=knee 2=hip 3=ankle
        auto& sp1=osimModel.updComponent<PathSpring>("/forceset/path_spring1");
        sp1.setStiffness(data.ints[4].val*4454.76);
	int spnum1=round(sp1.getStiffness()/4454.76);
        auto& sp2=osimModel.updComponent<PathSpring>("/forceset/path_spring2");
        sp2.setStiffness(data.ints[4].val*4454.76);
	int spnum2=round(sp2.getStiffness()/4454.76);
        auto& sp3=osimModel.updComponent<PathSpring>("/forceset/path_spring3");
        sp3.setStiffness(data.ints[4].val*4454.76);
	int spnum3=round(sp3.getStiffness()/4454.76);
        //auto& fset=osimModel.updForceSet();
        auto& cont1 = osimModel.updComponent<OpenSim::SmoothSphereHalfSpaceForce>(
                  "tforce");


	ProbeReporter* probeReporter = new ProbeReporter(&osimModel);
    	osimModel.addAnalysis(probeReporter);
    	ForceReporter* forceReporter = new ForceReporter(&osimModel);
    	osimModel.addAnalysis(forceReporter);
    	MuscleAnalysis* muscleReporter = new MuscleAnalysis(&osimModel);
    	osimModel.addAnalysis(muscleReporter);

	const MultibodySystem& system=osimModel.updMultibodySystem();
        system.addEventReporter(new MyReporter(system,.01,osimModel));
        State &osimState = osimModel.initializeState();
	//set initial state from first line of statesfile
	cout<<"apply first line of states...."<<endl;
	//cout.precision(15);
	for (int i=0; i<coloStates.numcols;i++)
	{osimModel.setStateVariableValue(osimState,coloStates.labels[i],st[i]);
         cout<<coloStates.labels[i]<<":"<<st[i]<<endl;}
	//cout<<"get end time from control file..."<<endl;

        double tf=time(time.nrow()-1);
	cout<<"Endtime:"<<tf<<endl;
	system.realizeTopology();
        Manager manager(osimModel);
        si.getQ().dump("Initial q's");
        cout<<"read model from:"<< modelFile<<endl;
        //cout<<"read traj solutionfrom:"<< controlsFile <<endl;
        //cout<<"read initial state:"<< statesFile <<endl;
try {
        manager.initialize(osimState);
        system.realize(osimState, Stage::Position);
        cout<<"tiploc:"<<osimModel.getMarkerSet()[6].getLocationInGround(osimState)<<endl;
        cout<<__LINE__<<endl;

        std::cout<<"Integrating from 0  to " << tf << std::endl;
        manager.setIntegratorAccuracy(0.5e-4);
        cout<<"minStep:"<<tf/((time.nrow()-1))<<endl;
	//manager.setIntegratorMinimumStepSize(.015);
	manager.setIntegratorMaximumStepSize(tf/500);
        osimModel.print("results/fwd_integ.osim");

        manager.integrate(tf);
        Vector_<SpatialVec>  forcesAtMInG;
        system.realize(manager.getState(), Stage::Acceleration);
        system.getMatterSubsystem().calcMobilizerReactionForces(manager.getState(),forcesAtMInG);
        SimTK::Vec3 comPos =
                system.getMatterSubsystem().calcSystemMassCenterLocationInGround(manager.getState());

        std::cout <<  manager.getState().getTime() <<":"<<
		cont1.getRecordValues( manager.getState())[0]<<
                ","<<cont1.getRecordValues( manager.getState())[1]<<"\t,"<< comPos[1]<<endl;


        cout<<"end integration at time:"<<manager.getState().getTime()<<endl;

        osimModel.getMultibodySystem().realize(manager.getState(), Stage::Velocity);
        osimModel.getMultibodySystem().realize(manager.getState(), Stage::Acceleration);
        Vec3 COM_position = osimModel.getMultibodySystem().getMatterSubsystem().
                        calcSystemMassCenterLocationInGround(manager.getState());
        Vec3 COM_velocity = osimModel.getMultibodySystem().getMatterSubsystem().
                        calcSystemMassCenterVelocityInGround(manager.getState());
        double g = -osimModel.getGravity()[1];
	double sysMass =  osimModel.getMultibodySystem().getMatterSubsystem()
					.calcSystemMass(manager.getState());

        double maxHeight = (COM_velocity[1]>0?COM_position[1] +
                                pow(COM_velocity[1], 2.0)/(2.0*g):0);

        auto statesTable = manager.getStatesTable();
        STOFileAdapter::write(statesTable, "results/fwd_states.sto");
        cout<<"Wrote states to results/fwd_states.sto"<<endl;
        forceReporter->getForceStorage().print("results/fwd_forces.sto");
	//const Set<const Actuator>& actSet = osimModel.get_ControllerSet().get(0).getActuatorSet();
        //auto& probe = osimModel.getProbeSet().get(0);
        //JointInternalPowerProbe*  jwip=dynamic_cast<JointInternalPowerProbe*>(&probe);
        //for (int i=0;i<4;i++){
        //double power=jwip->getProbeOutputs(manager.getState())(i);
	//	cout<<"power"<<i<<": "<<power<<endl;}
        //probeReporter->getProbeStorage().print("results/fwd0_probes.sto");
        //forceReporter->getForceStorage().print("results/fwd0_forces.sto");

        //cout.precision(15);
        cout<<"num of springs:[KHA]"<<spnum1<<","<<spnum2<<","<<spnum3<<endl;
	cout<<"pos:"<<COM_position<<" vel:"<<COM_velocity<<" mass:"<<sysMass<<endl;
	cout<<"jump:"<<maxHeight<<endl;
}
    catch (const std::exception& ex)
    {
        std::cout << "Exception in 4links_example: " << ex.what() << std::endl;
        return 1;
    }

                

return 0;


}
