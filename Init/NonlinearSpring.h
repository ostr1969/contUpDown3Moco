#ifndef OPENSIM_NONLINEARSPRING_H_
#define OPENSIM_NONLINEARSPRING_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  NonlinearSpring.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//==============================================================================
// INCLUDES
//==============================================================================
//#include "osimActuatorsDLL.h"
#define OSIMACTUATORS_API
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/Model/Force.h>

//==============================================================================
//==============================================================================
namespace OpenSim {

class Coordinate;

/**
 * A Force that exerts a generalized force based on spring-like
 * characteristics (stiffness and viscosity).  
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 2.0
 */
class OSIMACTUATORS_API NonlinearSpring : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(NonlinearSpring, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_OPTIONAL_PROPERTY(coordinate, std::string,
        "Name of the coordinate to which this force is applied.");
    OpenSim_DECLARE_PROPERTY(stiffness, double,
        "Spring stiffness.");
    OpenSim_DECLARE_PROPERTY(rest_length, double,
        "Coordinate value at which spring produces no force.");
    OpenSim_DECLARE_PROPERTY(viscosity, double,
        "Damping constant.");
    OpenSim_DECLARE_PROPERTY(force_vs_coordinate_spline, SimmSpline,
	"spline that defines the force curve vs. coordinate with ");
    OpenSim_DECLARE_OUTPUT(torq, double, getCurrentTorque, SimTK::Stage::Dynamics);


//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** This serves as default constructor or you can specify the coordinate
    name. A name of "" is treated as though unspecified. **/
NonlinearSpring(){
    setNull();
    constructProperties();
}
NonlinearSpring(const string& coordinateName)
{
    setNull();
    constructProperties();

    if (!coordinateName.empty())
        set_coordinate(coordinateName);
}
NonlinearSpring(const string& coordinateName,
                SimmSpline* ForceVsCoordinateSpline,double stiff){
        setNull();
        constructProperties();
        set_coordinate(coordinateName);
        set_force_vs_coordinate_spline(*ForceVsCoordinateSpline);
        set_stiffness(stiff);
}

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
void setForceVsCoordinateSpline(
                        SimmSpline* ForceVsCoordinateSpline) {
            set_force_vs_coordinate_spline(*ForceVsCoordinateSpline);
}

    // STIFFNESS
void setStiffness(double aStiffness)
{ set_stiffness(aStiffness); }
double getStiffness() const
{ return get_stiffness(); }

    // REST LENGTH
void setRestLength(double aRestLength)
{ set_rest_length(aRestLength); }
double getRestLength() const
{ return get_rest_length(); }

    // VISCOSITY
void setViscosity(double aViscosity)
{ set_viscosity(aViscosity); }

double getViscosity() const
{ return get_viscosity(); }

    double getCurrentTorque(const SimTK::State& s) const {
		            return computeForceMagnitude(s);}
    /** 
     * Methods to query a Force for the value actually applied during simulation
     * The names of the quantities (column labels) is returned by this first function
     * getRecordLabels()
     */
OpenSim::Array<std::string> getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName()+"_Force");
    return labels;
}

    /**
     * Given SimTK::State object extract all the values necessary to report forces, application location
     * frame, etc. used in conjunction with getRecordLabels and should return same size Array
     */
OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
    OpenSim::Array<double> values(1);
    values.append(computeForceMagnitude(state));
    return values;
};


    //--------------------------------------------------------------------------
    // COMPUTATIONS
protected:
    // Force interface.
void computeForce(const SimTK::State& s,
                                  SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                                  SimTK::Vector& generalizedForces) const
{ if( !_model || !_coord ) return;
// FORCE
    applyGeneralizedForce(s, *_coord, computeForceMagnitude(s), generalizedForces);
}

    
    // ModelComponent interface.
void extendAddToSystem(SimTK::MultibodySystem& system) const
{ Super::extendAddToSystem(system);
    if (_model) {
        NonlinearSpring* mthis =
            const_cast<NonlinearSpring*>(this);
        mthis->_coord = &_model->updCoordinateSet().get(get_coordinate());
    } }


    // Setup method to initialize coordinate reference
void extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
    _coord = &model.updCoordinateSet().get(get_coordinate());
}


private:
void setNull()
{
    setAuthors("barak based on Frank C. Anderson ");
}
void constructProperties()
{
    constructProperty_coordinate();
    constructProperty_stiffness(1.0);
    constructProperty_rest_length(0.0);
    constructProperty_viscosity(0.0);
    SimmSpline ss;
    constructProperty_force_vs_coordinate_spline(ss);
}

double computeForceMagnitude(const SimTK::State& s) const
{ double q = _coord->getValue(s);
    //double speed =  _coord->getSpeedValue(s);
    //double force = -getStiffness()*(q - get_rest_length())
    //                    - get_viscosity()*speed;
    double force = get_force_vs_coordinate_spline().calcValue(SimTK::Vector(1, q));
    return force*getStiffness(); }

    // Set the Coordinate pointer, and set the corresponding name property
    // to match.
    void setCoordinate(Coordinate* coordinate);
    Coordinate* getCoordinate() const;
const SimmSpline* getForceVsCoordinateSpline() {
            return &get_force_vs_coordinate_spline();
}

    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

    // Corresponding generalized coordinate to which the coordinate actuator
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;

    //==============================================================================
};  // END of class NonlinearSpring

}; //namespace
//==============================================================================
//==============================================================================


#endif // OPENSIM_NONLINEARSPRING_H_
