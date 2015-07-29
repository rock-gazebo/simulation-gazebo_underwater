#include "GazeboUnderwater.hpp"
#include <gazebo/common/Exception.hh>

using namespace std;
using namespace gazebo;

namespace gazebo_underwater
{
    GazeboUnderwater::GazeboUnderwater() : use_quadratic_damp(false)
    {
    }

    GazeboUnderwater::~GazeboUnderwater()
    {
    }

    void GazeboUnderwater::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        gzmsg << "GazeboUnderwater: Loading underwater environment." << endl;

        world = _world;
        sdf = _sdf;

        loadParameters();

        // Each simulation step the Update method is called to update the simulated sensors and actuators
        eventHandler.push_back(
                event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&GazeboUnderwater::updateBegin,this, _1)));
    }

    template <class T> 
    T GazeboUnderwater::getParameter(string parameter_name, string dimension, T default_value)
    {
        T var = default_value;
        if(sdf->HasElement(parameter_name.c_str()))
        {
            var = sdf->Get< T >(parameter_name.c_str());
            gzmsg << "GazeboUnderwater: " + parameter_name + ": (" << var << ") "
                    + dimension  << endl;
        }else{
            gzmsg << "GazeboUnderwater: " + parameter_name + ": using default ("
                    << default_value << ") " + dimension << endl;
        }
        return var;
    }

    void GazeboUnderwater::loadParameters(void)
    {
        // Test if model_name is defined in the world file
        if(sdf->HasElement("model_name"))
        {
            // Test if model_name name a model loaded in gazebo
            model = world->GetModel( sdf->Get<string>("model_name") );
            if(!model)
            {
                string msg ="GazeboUnderwater: model " + sdf->Get<string>("model_name") +
                        " not found in gazebo world: " + world->GetName() + ". ";
                string available = "Known models are:";
                gazebo::physics::Model_V models = world->GetModels();
                for (size_t i = 0; i < models.size(); ++i)
                    available += " " + models[i]->GetName();
                msg += available;
                gzthrow(msg);
            }else{
                gzmsg << "GazeboUnderwater: model: " << model->GetName() << endl;
            }
        }else{
            gzthrow("GazeboUnderwater: model_name not defined in world file !");
        }

        if(sdf->HasElement("link_name"))
        {
            link = model->GetLink( sdf->Get<string>("link_name") );
            if (!link) {
                string msg = "GazeboUnderwater: link " + sdf->Get<string>("link_name")
                        + " not found in model " + model->GetName();
                gzthrow(msg);
            }else{
                gzmsg << "GazeboUnderwater: link: " << link->GetName() << endl;
                physics::InertialPtr modelInertia = link->GetInertial();
                gzmsg << "GazeboUnderwater: link mass: " << modelInertia->GetMass() << " kg" << endl;
                gzmsg << "GazeboUnderwater: link weight: " <<
                    - modelInertia->GetMass() * world->GetPhysicsEngine()->GetGravity().z << " N" << endl;
            }
        }else{
            gzthrow("GazeboUnderwater: link_name not defined in world file !");
        }

        math::Box linkBoudingBox = link->GetBoundingBox();

        waterLevel = getParameter<double>("water_level","meters", 0.0);
        fluidVelocity = getParameter<math::Vector3>("fluid_velocity","m/s",math::Vector3(0,0,0));
        densityOfFluid = getParameter<double>("fluid_density","kg/m3", 1027);
        linearDragCoefficients = getParameter<math::Vector3>("linear_drag_coefficients","dimensionless",
                math::Vector3(1,1,1));
        // damp coefficients take precedence over drag coefficients
        if( sdf->HasElement("quadratic_damp_coefficients") )
        {
            use_quadratic_damp = true;
            quadraticDampCoefficients = getParameter<math::Vector3>("quadratic_damp_coefficients","N.s2/m2",
                    math::Vector3(75,75,75));
        }else{
            quadraticDragCoefficients = getParameter<math::Vector3>("quadratic_drag_coefficients","dimensionless",
                math::Vector3(.5,.5,.5));
        }
        volume = getParameter<double>("volume","meter3",linkBoudingBox.GetXLength()*linkBoudingBox.GetYLength()*linkBoudingBox.GetZLength());
        // buoyancy must be the buoyancy when the model is completely submersed
        buoyancy = getParameter<double>("buoyancy","N", volume * densityOfFluid * world->GetPhysicsEngine()->GetGravity().GetLength());
        centerOfBuoyancy = getParameter<math::Vector3>("center_of_buoyancy","meters",
                math::Vector3(0, 0, 0.15));

        // If side_areas are not given in world file we use the bouding box dimensions to calculate it
        sideAreas = getParameter<math::Vector3>("side_areas","meter2",
                math::Vector3(linkBoudingBox.GetYLength() * linkBoudingBox.GetZLength(),
                    linkBoudingBox.GetXLength() * linkBoudingBox.GetZLength(),
                    linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength()));
        if( sideAreas == math::Vector3(0.0,0.0,0.0) )
            gzthrow("GazeboUnderwater: side_areas cannot be (0.0, 0.0, 0.0).");
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
        if( distanceToSurface > 0 )
        {
            math::Vector3 velocityDifference = link->GetWorldCoGLinearVel() - fluidVelocity;
            math::Vector3 angularVelocity = link->GetWorldAngularVel();

            // Linear drag
            math::Vector3 linearDrag = - linearDragCoefficients * velocityDifference;
            link->AddForceAtWorldPosition(linearDrag,cogPosition);

            math::Vector3 linearAngularDrag = - linearDragCoefficients * angularVelocity;
            link->AddTorque(linearAngularDrag);

            // Quadratic drag
            if(use_quadratic_damp)
            {
                // damp coefficients
                math::Vector3 quadraticDrag = - quadraticDampCoefficients *
                         velocityDifference.GetAbs() * velocityDifference;
                link->AddForceAtWorldPosition(quadraticDrag,cogPosition);

                math::Vector3 quadraticAngularDrag = - quadraticDampCoefficients *
                         angularVelocity.GetAbs() * angularVelocity;
                link->AddTorque(quadraticAngularDrag);
            }else{
                // drag coefficients
                math::Vector3 quadraticDrag = - 0.5 * densityOfFluid * sideAreas * quadraticDragCoefficients *
                         velocityDifference.GetAbs() * velocityDifference;
                link->AddForceAtWorldPosition(quadraticDrag,cogPosition);

                math::Vector3 quadraticAngularDrag = - 0.5 * densityOfFluid * sideAreas * quadraticDragCoefficients *
                         angularVelocity.GetAbs() * angularVelocity;
                link->AddTorque(quadraticAngularDrag);
            }
        }
    }


    void GazeboUnderwater::applyBuoyancy()
    {
        math::Vector3 cogPosition = link->GetWorldCoGPose().pos;
        double distanceToSurface = waterLevel - cogPosition.z;
        math::Vector3 linkBuoyancy;

        double submersedVolume = calculateSubmersedVolume(distanceToSurface);
        // The buoyancy is proportional no the submersed volume
        linkBuoyancy = math::Vector3(0,0,submersedVolume * abs(buoyancy));

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
