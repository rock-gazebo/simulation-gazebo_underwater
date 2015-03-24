#include "GazeboUnderwater.hpp"

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
    T GazeboUnderwater::getParameter(std::string _parameter_name, std::string dimension, T default_value)
    {
        T var = default_value;
        if(sdf->HasElement(_parameter_name.c_str()))
        {
            var = sdf->Get< T >(_parameter_name.c_str());
            gzmsg << "GazeboUnderwater: " + _parameter_name + ": (" << var << ") "
                    + dimension  << std::endl;
        }else{
            gzmsg << "GazeboUnderwater: " + _parameter_name + ": using default ("
                    << default_value << ") " + dimension << std::endl;
        }
        return var;
    }

    void GazeboUnderwater::loadParameters(void)
    {
        if(sdf->HasElement("model_name"))
        {
            model = world->GetModel( sdf->Get<std::string>("model_name") );  
            gzmsg << "GazeboUnderwater: found model: " << model->GetName() << std::endl;
        }else{
            gzmsg << "GazeboUnderwater: model_name to apply buoyancy not defined !" << std::endl;
        }

        link = model->GetLink( sdf->Get<std::string>("link_name") );
        if( link ){
            gzmsg << "GazeboUnderwater: found link: " << link->GetName() << std::endl;
            physics::InertialPtr modelInertia = link->GetInertial();
            gzmsg << "GazeboUnderwater: link mass: " << modelInertia->GetMass() << std::endl;
        }else{
            gzmsg << "GazeboUnderwater: no model link matches link_name !" << std::endl;
        }

        size = getParameter<math::Vector3>("size","meters",
                math::Vector3(1, 1, 1));
        centerOfBuoyancy = getParameter<math::Vector3>("center_of_buoyancy","meters",
                math::Vector3(0, 0, 0.15));
        fluidVelocity = getParameter<math::Vector3>("fluid_velocity","m/s",
                math::Vector3(0,0,0));
        viscousDamping = getParameter<math::Vector3>("viscous_damping","dimensionless",
                math::Vector3(20,30,50));
        waterLevel = getParameter<double>("water_level","meters", 2.0);
        densityOfFluid = getParameter<double>("fluid_density","kg/m3", 1000);
        // buoyancy must be the buoyancy when the model is completely submersed
        buoyancy = getParameter<double>("buoyancy","N",0);
    }

    void GazeboUnderwater::updateBegin(common::UpdateInfo const& info)
    {
        if(link)
        {
            applyBuoyancy();
            applyViscousFriction();
        }
    }

    void GazeboUnderwater::applyViscousFriction()
    {
        math::Vector3 cobPosition = link->GetWorldCoGPose().pos +
                link->GetWorldCoGPose().rot.RotateVector(centerOfBuoyancy);

        // Calculates dynamic viscous damp
        math::Vector3 velocityDifference = link->GetWorldCoGPose().rot.RotateVector(
                link->GetWorldLinearVel() - fluidVelocity);
        math::Vector3 viscousDrag = - link->GetWorldCoGPose().rot.RotateVector(
                viscousDamping * velocityDifference );
        link->AddForceAtWorldPosition(viscousDrag,cobPosition);
    }


    void GazeboUnderwater::applyBuoyancy()
    {
        math::Vector3 cobPosition = link->GetWorldCoGPose().pos +
                link->GetWorldCoGPose().rot.RotateVector(centerOfBuoyancy);
        double distanceToSurface = waterLevel - cobPosition.z;
        math::Vector3 linkBuoyancy;

        // buoyancy value is used, if defined (!= 0)
        if( abs(buoyancy) )
        {
            math::Box linkBoudingBox = link->GetBoundingBox();
            double submersedVolume = calculateSubmersedVolume(linkBoudingBox.GetXLength(),linkBoudingBox.GetYLength(),
                    linkBoudingBox.GetZLength(),distanceToSurface);
            double linkVolume = linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength() * linkBoudingBox.GetZLength();
            submersedVolume = submersedVolume/linkVolume;
            // The buoyancy is proportional no the submersed volume
            linkBuoyancy = - submersedVolume * abs(buoyancy) * world->GetPhysicsEngine()->GetGravity();
        }else{
            double submersedVolume = calculateSubmersedVolume(size.x,size.y,size.z,distanceToSurface);
           // The buoyancy opposes gravity	=> it is negative.
            linkBuoyancy = - submersedVolume * densityOfFluid * world->GetPhysicsEngine()->GetGravity();
        }
        link->AddForceAtWorldPosition(linkBuoyancy,cobPosition);
    }

    double GazeboUnderwater::calculateSubmersedVolume(double x, double y, double z,double distanceToSurface)
    {
        double submersedVolume = 0.0;
        if(distanceToSurface <= 0.0)
        {
            submersedVolume = 0.0;
        }else{
            if(distanceToSurface <= (z)/2.0 )
            {
                submersedVolume = x * y * ( z/2.0 + distanceToSurface );
            }else{
                submersedVolume = x * y * z;
            }
        }
        return submersedVolume;
    }
}
