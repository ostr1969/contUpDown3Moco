/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoJumpGoal.cpp                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//#include "MocoGoal.h"

//#include <Moco/MocoGoal/MocoGoal.h>
#include <OpenSim/OpenSim.h>
#include "MocoJumpGoal.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;


void MocoJumpGoal::initializeOnModelImpl(const Model& model ) const {
    setRequirements(1, 3);
    
}

void MocoJumpGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
	 Vector_<SpatialVec>  forcesAtMInG;
        getModel().realizeVelocity(input.state);
    //std::vector<std::string> contact1;
    //contact1.push_back("tforce");
    //contact1.push_back("aforce");

	 Array<double> f1=getModel().getComponent<Force>("tforce").getRecordValues(input.state);
	 Array<double> f2=getModel().getComponent<Force>("aforce").getRecordValues(input.state);
	 auto& tip=getModel().getMarkerSet()[6];
	 double tipy=tip.getLocationInGround(input.state)[1];
     double f=f1[1];//+f2[1];
     double fn,t1,t2,k;
    /*if (f>=100){
    t1=0;t2=5000;double y1=0,y2=5;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    fn=k*k*(3-2*k)*(y2-y1)+y1;
    fn=0;}
    if (f<50){
    t1=0;t2=50;double y1=60,y2=0;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    fn=k*k*(3-2*k)*(y2-y1)+y1;
    //cout<<"f:"<<f<<" fn:"<<fn<<endl;
            }
    */
    integrand = tipy;
    //integrand = f>20?0:1;

}


void MocoJumpGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
	// Vector_<SpatialVec>  forcesAtMInG;
       //SimTK::Real timeInitial = input.initial_state.getTime();
       // SimTK::Real timeFinal = input.final_state.getTime();
        //SimTK::Vec3 comInitialV =
        //        model.calcMassCenterVelocity(input.initial_state);
        SimTK::Vec3 comFinalV =
                getModel().calcMassCenterVelocity(input.final_state);
        SimTK::Vec3 comFinalP =
                getModel().calcMassCenterPosition(input.final_state);
        auto& tip=getModel().getMarkerSet()[6];

    //cout<<tip.getLocationInGround(input.initial_state)[1]<<","<<tip.getLocationInGround(input.final_state)[1]<<endl;;

    //double t1=-1,t2=1,
    double vy=comFinalV(1);
    //double k=std::max(0.,std::min(1.,(vy-t1)/(t2-t1)));
    //double dirac=k*k*(3-2*k);
    //cost[0]=-vy*vy/2./9.81*dirac-comFinalP(1);
    cost[0]=-vy-comFinalP(1);//not necessery the hight, only highest v and h
    double tipinitx=tip.getLocationInGround(input.initial_state)[0];
    double tipinity=tip.getLocationInGround(input.initial_state)[1];
    //Vec3 tipinitLoc=tip.getLocationInGround(input.initial_state);
    //cout<<tip.getName()<<tipinitLoc<<"  ";
    double tipMovx=tip.getLocationInGround(input.final_state)[0]-tipinitx;
    double tipy0=tipinity-0.0174689;
    //k=std::max(0.,std::min(1.,(tipMovx-t1)/(t2-t1)));
    //dirac=k*k*(3-2*k);
    cost[1]=tipMovx*tipMovx*100000;
    //cost[2]=tipy0*tipy0*100000;
    double t1=-1,t2=1;
    double k=std::max(0.,std::min(1.,(vy-t1)/(t2-t1)));
    double dirac=k*k*(3-2*k);
    cost[2] = input.integral*100;
    //cout<<"cost0:"<<cost[0]<<" cost1:"<<cost[1]<<" cost2:"<<cost[2]<<endl;

/*	getModel().realizeAcceleration(input.final_state);
	 getModel().getMultibodySystem().getMatterSubsystem().
			calcMobilizerReactionForces( input.final_state,forcesAtMInG);
    double f=forcesAtMInG[0][1][1];
    if (f>=0){
    t1=0;t2=28000;double y1=0,y2=28;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    cost[1]=k*k*(3-2*k)*(y2-y1)+y1;}
    if (f<0){
    t1=-28000;t2=0;double y1=28,y2=0;
    k=std::max(0.,std::min(1.,(f-t1)/(t2-t1)));
    cost[1]=k*k*(3-2*k)*(y2-y1)+y1;}
    cost[0] = input.integral;*/
}
