#include "GazeboUnderwater.hpp"
#include <gazebo/common/Exception.hh>

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
        // Test if model_name is defined in the world file
        if(sdf->HasElement("model_name"))
        {
            // Test if model_name name a model loaded in gazebo
            model = world->GetModel( sdf->Get<std::string>("model_name") );  
            if(model)
            {
                gzmsg << "GazeboUnderwater: model: " << model->GetName() << std::endl;
            }else{
                std::string msg = "GazeboUnderwater: model " + sdf->Get<std::string>("model_name")
                        + " not found in gazebo world " + world->GetName();
                gzthrow(msg);
            }
        }else{
            gzthrow("GazeboUnderwater: model_name not defined in world file !");
        }

        if(sdf->HasElement("link_name"))
        {
            link = model->GetLink( sdf->Get<std::string>("link_name") );
            if( link ){
                gzmsg << "GazeboUnderwater: link: " << link->GetName() << std::endl;
                physics::InertialPtr modelInertia = link->GetInertial();
                gzmsg << "GazeboUnderwater: link mass: " << modelInertia->GetMass() << std::endl;
            }else{
                std::string msg = "GazeboUnderwater: link " + sdf->Get<std::string>("link_name")
                        + " not found in model " + model->GetName();
                gzthrow(msg);
            }
        }else{
            gzthrow("GazeboUnderwater: link_name not defined in world file !");
        }

        waterLevel = getParameter<double>("water_level","meters", 2.0);
        fluidVelocity = getParameter<math::Vector3>("fluid_velocity","m/s",math::Vector3(0,0,0));
        densityOfFluid = getParameter<double>("fluid_density","kg/m3", 1027);
        dragCoefficient = getParameter<math::Vector3>("drag_coefficient","dimensionless",
                math::Vector3(1,1,1));
        // buoyancy must be the buoyancy when the model is completely submersed
        buoyancy = getParameter<double>("buoyancy","N",0);
        centerOfBuoyancy = getParameter<math::Vector3>("center_of_buoyancy","meters",
                math::Vector3(0, 0, 0.15));
        sideAreas = getParameter<math::Vector3>("side_areas","meter2",
                math::Vector3(0.5,0.5,0.5));
        volume = getParameter<double>("volume","meter3",1);
    }

    void GazeboUnderwater::updateBegin(common::UpdateInfo const& info)
    {
        applyBuoyancy();
        applyViscousDrag();
    }

    void GazeboUnderwater::applyViscousDrag()
    {
        math::Vector3 cogPosition = link->GetWorldCoGPose().pos;
        double distanceToSurface = waterLevel - cogPosition.z;
        if(distanceToSurface > 0 )
        {
            math::Vector3 velocityDifference = link->GetWorldCoGLinearVel() - fluidVelocity;
            math::Vector3 viscousDrag = - 0.5 * densityOfFluid * sideAreas * dragCoefficient *
                     velocityDifference.GetAbs() * velocityDifference;
            link->AddForceAtWorldPosition(viscousDrag,cogPosition);

            math::Vector3 angularVelocity = link->GetWorldAngularVel();
            math::Vector3 angularDrag = - 0.5 * densityOfFluid * sideAreas * dragCoefficient *
                     angularVelocity.GetAbs() * angularVelocity;
            link->AddTorque(angularDrag);
        }
    }


    void GazeboUnderwater::applyBuoyancy()
    {
        math::Vector3 cogPosition = link->GetWorldCoGPose().pos;
        double distanceToSurface = waterLevel - cogPosition.z;
        math::Vector3 linkBuoyancy;

        // buoyancy value is used if defined (!= 0)
        if( abs(buoyancy) )
        {
            double submersedVolume = calculateSubmersedVolume(distanceToSurface);
            // The buoyancy is proportional no the submersed volume
            linkBuoyancy = math::Vector3(0,0,submersedVolume * abs(buoyancy));
        }else{
            double submersedVolume = calculateSubmersedVolume(distanceToSurface);
            // The buoyancy opposes gravity
            linkBuoyancy = - submersedVolume * volume * densityOfFluid * world->GetPhysicsEngine()->GetGravity();
        }

        math::Vector3 cobPosition = link->GetWorldCoGPose().pos +
                link->GetWorldCoGPose().rot.RotateVector(centerOfBuoyancy);
        link->AddForceAtWorldPosition(linkBuoyancy,cobPosition);
    }

    double GazeboUnderwater::calculateSubmersedVolume(double distanceToSurface)
    {
        math::Box linkBoudingBox = link->GetBoundingBox();
        double linkVolume = linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength() * linkBoudingBox.GetZLength();
        double submersedVolume = 0.0;

        if(distanceToSurface <= 0.0)
        {
            submersedVolume = 0.0;
        }else{
            if(distanceToSurface <= (linkBoudingBox.GetZLength())/2.0 )
            {
                submersedVolume = linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength() *
                        ( linkBoudingBox.GetZLength()/2.0 + distanceToSurface );
            }else{
                submersedVolume = linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength() *
                        linkBoudingBox.GetZLength();
            }
        }
        // The submersed volume is given in percentage
        submersedVolume = submersedVolume/linkVolume;
        return submersedVolume;
    }
}
