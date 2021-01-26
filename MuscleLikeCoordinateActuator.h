#ifndef MOCO_MUSCLELIKECOORDINATEACTUATOR_H
#define MOCO_MUSCLELIKECOORDINATEACTUATOR_H 1
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MuscleLikeCoordinateActuator.h                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Carmichael Ong                                                  *
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

//#include "osimMocoDLL.h"
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/osimCommon.h>

namespace OpenSim { 

/// This class derives from CoordinateActuator but has additional properties 
/// that mimic muscle-like properties according to the coordinate. The 
/// generalized force that is applied along the coordinate is calculated by a 
/// gain, an excitation (control signal), a function describing force vs 
/// coordinate, and a function describing a force vs the derivative of the 
/// coordinate:
///
///              F = (gain)*(excitation)*F_max(q)*F_vel(qdot)
///
/// The actuator expects two different function for the F_max curves, one for 
/// when excitation is positive and one for when excitation is negative. Each 
/// of the curves must be given as an Opensim::Function.
///
/// The F_vel curve is described with a single parameter, qdot_max, as 
/// described in Blake Ashby's thesis (2004). For positive excitations, 
/// the function returns:
///	    0,								               for qdot > qdot_max 
///		1.5,					      for qdot < (-0.35678917232)*qdot_max
///	    1 - (1.01750751592)*atan(1.5*qdot/qdot_max),	         otherwise
/// For negative excitations, the inequalities flip and the sign for qdot_max 
/// flips.
///
/// @author Carmichael Ong
///
class MuscleLikeCoordinateActuator : public CoordinateActuator {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleLikeCoordinateActuator, 
        CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(qdot_max, double,
        "The velocity at which the actuator produces zero force.");
    OpenSim_DECLARE_PROPERTY(pos_force_vs_coordinate_function, Function,
        "Function that defines the optimal force curve vs. coordinate with "
        "positive activation.");
    OpenSim_DECLARE_PROPERTY(neg_force_vs_coordinate_function, Function,
        "Function that defines the optimal force curve vs. coordinate with "
        "negative activation.");

    /// Default constructor leaves coordinate name unspecified, or you can
    /// provide it. 
    MuscleLikeCoordinateActuator(const std::string& coordinateName="");

    /// Convience constructor.
    MuscleLikeCoordinateActuator(const std::string& coordinateName, 
            double optimalForce, 
            Function* posForceVsCoordinateFunction, 
            Function* negForceVsCoordinateFunction, 
            double qdot_max);

    /// Get and set methods.
    void setPosForceVsCoordinateFunction(
            Function* posForceVsCoordinateFunction);
    const Function* getPosForceVsCoordinateFunction();

    void setNegForceVsCoordinateFunction(
            Function* negForceVsCoordinateFunction);
    const Function* getNegForceVsCoordinateFunction();

    void setMaxVelocity(double qdot_max);
    double getMaxVelocity();

private:
    /// Compute all quantities necessary for applying the actuator force to the
    /// model.
    double computeActuation(const SimTK::State& s) const override;

    /// Helper functions to calculate F_max and F_vel for computeActuation.
    double getForceVsCoordinateFunctionValue(const SimTK::State& s) const;
    double getVelocityMultiplier(const SimTK::State& s) const;

    void setNull();
    void constructProperties();

}; // class MuscleLikeCoordinateActuator
} //namespace Opensim
#endif // MOCO_MUSCLELIKECOORDINATEACTUATOR_H
#ifndef Musclecpp
using namespace OpenSim;

class
    ActivationMuscleLikeCoordinateActuator :
            public MuscleLikeCoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ActivationMuscleLikeCoordinateActuator,
        MuscleLikeCoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Larger value means activation can change more rapidly "
        "(units: seconds).");
    OpenSim_DECLARE_PROPERTY(dactivation_time_constant, double,
        "Larger value means deactivation can change more rapidly "
        "(units: seconds).");

    OpenSim_DECLARE_PROPERTY(default_activation, double,
        "Value of activation in the default state returned by initSystem().");

    ActivationMuscleLikeCoordinateActuator() {
        constructProperties();
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activation", SimTK::Stage::Dynamics);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "activation", get_default_activation());
    }
    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_activation(getStateVariableValue(s, "activation"));
    }

    // TODO no need to do clamping, etc; CoordinateActuator is bidirectional.
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
	SimTK::Real adot=0.;
        const auto& tau = get_activation_time_constant();
        const auto& taud = get_dactivation_time_constant();	
        const auto& u = getControl(s);
        const auto& a = getStateVariableValue(s, "activation");
        if (u*u>=a*a)  adot = (u - a) / tau;
        if (u*u<a*a)   adot = (u - a) / taud;
        //const SimTK::Real adot = (u - a) / tau;
        setStateVariableDerivativeValue(s, "activation", adot);
    }

    double computeActuation(const SimTK::State& s) const override {
        return getStateVariableValue(s, "activation") * getOptimalForce();
    }
private:
    void constructProperties() {
        constructProperty_activation_time_constant(0.011);
        constructProperty_dactivation_time_constant(0.068);
        constructProperty_default_activation(0.011);
    }
};


void addMuscleLikeCoordinateActuator(Model& model, std::string coordName,
        double act,double dact, PolynomialFunction *posFunc,
	PolynomialFunction *negFunc,double maxvel) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new ActivationMuscleLikeCoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->set_activation_time_constant(act);
    actu->set_dactivation_time_constant(dact);
    actu->setOptimalForce(1.0);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    actu->set_qdot_max(maxvel);

    /*auto* posFunc = new PolynomialFunction();
    posFunc->setName("pos_force_vs_coordinate_function");
    auto* negFunc = new PolynomialFunction();
    negFunc->setName("neg_force_vs_coordinate_function");

    // Polynomial coefficients from Carmichael Ong's SimTK project
    // "Predictive Simulation of Standing Long Jumps". Borrowed from the
    // "AshbyModel_twoConstraints.osim" model.
    if (coordName.find("q3") != std::string::npos) {
        posFunc->setCoefficients(
            SimTK::Vector(SimTK::Vec4(27.175, -163.26, 146.58, 203.88)));
	//0.742339862253223  -4.542864698532929   8.732127637752997  -4.978034773947368
        negFunc->setCoefficients(
            SimTK::Vector(SimTK::Vec4(-15.492, 0.99992, 188.07, 326.63)));
	//0.078819723367340  -0.543912588603953   1.228360684622656  -0.802096947240235
        actu->set_qdot_max(20.0);
    } else if (coordName.find("q2") != std::string::npos) {
        posFunc->setCoefficients(
            SimTK::Vector(SimTK::Vec4(11.285, -135.23, 282.53, 238.77)));
	//-0.055604338040457   0.157147151244882   0.084912491222937   0.175916199240123
        negFunc->setCoefficients(
            SimTK::Vector(SimTK::Vec4(69.248, -454.99, 712.19, 203.07)));
	//-0.011273696302047  -0.029307513807120   0.233919713013925   0.141071418920201
        actu->set_qdot_max(18.0);
    } else if (coordName.find("q1") != std::string::npos) {
        posFunc->setCoefficients(
            SimTK::Vector(SimTK::Vec4(-80.378, -173.56, -102.12, 91.211)));
	// 0.015003063668593  -0.141656912179625   0.257201902753094   0.452266806484893
        negFunc->setCoefficients(
            SimTK::Vector(SimTK::Vec4(-748.14, -1054.1, 38.366, 407.2)));
	// -0.026201061331305   0.086665297019733   0.086939026741038  -0.111846043013201
        actu->set_qdot_max(16.0);
    }*/

    actu->setPosForceVsCoordinateFunction(posFunc);
    actu->setNegForceVsCoordinateFunction(negFunc);
    model.addComponent(actu);
}

#endif


