#include "GazeboUnderwater.hpp"
#include "gazebo/math/Vector3.hh"

using namespace gazebo;

namespace gazebo_underwater
{
    void GazeboUnderwater::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        gzmsg << "GazeboUnderwater: Loading underwater environment." << std::endl;

        world = _world; 
        sdf = _sdf; 

        loadParameters(); 

        // Each simulation step the Update method is called to update the simulated sensors and actuators
        eventHandler.push_back(
                event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&GazeboUnderwater::updateBegin,this, _1)));
    }


    template <class T> 
    T GazeboUnderwater::getParameter(std::string _parameter_name, T default_value)
    {
        T var = default_value;
        if(sdf->HasElement(_parameter_name.c_str()))
        {
            var = sdf->Get< T >(_parameter_name.c_str());
            gzmsg << "GazeboUnderwater: " + _parameter_name + ": " << var << std::endl; 
        }        
        else
        {
            gzmsg << "GazeboUnderwater: " + _parameter_name + ": using default " << default_value << std::endl; 
        }
        return var;
    }


    void GazeboUnderwater::loadParameters(void)
    {
        // If these parameters are not found the simulation must stop. 
        if(sdf->HasElement("model_name"))   
        {
            model = world->GetModel( sdf->Get<std::string>("model_name") );  
            gzmsg << "GazeboUnderwater: found model:" << model->GetName() << std::endl;
        }else{
            gzmsg << "GazeboUnderwater: model not found. Quit simulation... " << std::endl;
        }

        size = getParameter<math::Vector3>("size",
                math::Vector3(1, 1, 1));
        centerOfBuoyancy = getParameter<math::Vector3>("center_of_buoyancy",
                math::Vector3(0, 0, 0.1));
        waterLevel      = getParameter<double>("water_level", 0.0);
        viscousDamping  = getParameter<double>("viscous_damping", 0.0);
        densityOfFluid = getParameter<double>("liquid_density", 1000);
    }

    void GazeboUnderwater::updateBegin(common::UpdateInfo const& info)
    {
        physics::LinkPtr link = model->GetLink( sdf->Get<std::string>("link_name") ); 
        applyBuoyancy(link);
        applyViscousFriction(link);
    }

    void GazeboUnderwater::applyViscousFriction(physics::LinkPtr link)
    {
        // Calculates dynamic viscous damp
        math::Vector3 fluidVelocityRelative =
            link->GetWorldPose().rot.RotateVectorReverse(fluidVelocity);
        math::Vector3 velocityDifference =
            link->GetRelativeLinearVel() - fluidVelocityRelative;
        gzmsg << "viscous friction: " << velocityDifference * viscousDamping << std::endl;
        link->AddForce(- velocityDifference * viscousDamping);
    }


    void GazeboUnderwater::applyBuoyancy(physics::LinkPtr link)
    {
        math::Vector3 cobWorldPosition = link->GetWorldCoGPose().pos + 
            link->GetWorldCoGPose().rot.RotateVector(centerOfBuoyancy);		

        // link_buoyancy goes to zero when the model link is above the surface. 
        // it depends on the volume submersed
        double distanceToSurface = waterLevel - cobWorldPosition.z;

        double submersedVolume = size.x * size.y * std::min(distanceToSurface, size.z);
        if (submersedVolume <= 0)
            submersedVolume = 0;

        // Apply buoyancy to links in underwater environment
        math::Vector3 gravityRelative = link->GetWorldPose().rot.
            RotateVectorReverse(world->GetPhysicsEngine()->GetGravity());
        // The buoyancy opposes gravity	=> it is negative.		
        math::Vector3 buoyancyRelative = - submersedVolume * densityOfFluid * gravityRelative;
        link->AddForceAtRelativePosition(buoyancyRelative, centerOfBuoyancy);
    }
}


