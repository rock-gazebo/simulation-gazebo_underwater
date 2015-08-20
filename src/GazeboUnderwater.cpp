#include "GazeboUnderwater.hpp"
#include <gazebo/common/Exception.hh>

using namespace std;
using namespace gazebo;

namespace gazebo_underwater
{
    GazeboUnderwater::GazeboUnderwater()
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

        model = getModel(_world, _sdf);
        link  = getReferenceLink(model, _sdf);
        loadParameters();

        // Each simulation step the Update method is called to update the simulated sensors and actuators
        eventHandler.push_back(
                event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&GazeboUnderwater::updateBegin,this, _1)));
    }

    template <class T>
    T GazeboUnderwater::getParameter(string parameter_name, string dimension, T default_value) const
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

    physics::ModelPtr GazeboUnderwater::getModel(physics::WorldPtr world, sdf::ElementPtr sdf) const
    {
        if (!sdf->HasElement("model_name"))
            gzthrow("GazeboUnderwater: model_name not defined in world file !");

        physics::ModelPtr model = world->GetModel( sdf->Get<string>("model_name") );
        if (!model)
        {
            string msg ="GazeboUnderwater: model " + sdf->Get<string>("model_name") +
                    " not found in gazebo world: " + world->GetName() + ". ";
            string available = "Known models are:";
            gazebo::physics::Model_V models = world->GetModels();
            for (size_t i = 0; i < models.size(); ++i)
                available += " " + models[i]->GetName();
            msg += available;
            gzthrow(msg);
        }
        gzmsg << "GazeboUnderwater: model: " << model->GetName() << endl;
        return model;
    }

    physics::LinkPtr GazeboUnderwater::getReferenceLink(physics::ModelPtr model, sdf::ElementPtr sdf) const
    {
        if(sdf->HasElement("link_name"))
        {
            physics::LinkPtr link = model->GetLink( sdf->Get<string>("link_name") );
            gzmsg << "GazeboUnderwater: reference link: " << link->GetName() << endl;
            if (!link) {
                string msg = "GazeboUnderwater: link " + sdf->Get<string>("link_name")
                        + " not found in model " + model->GetName();
                gzthrow(msg);
            }
            return link;
        }else if (model->GetLinks().empty()) {
            gzthrow("GazeboUnderwater: no link defined in model");
        }else{
            physics::LinkPtr link = model->GetLinks().front();
            gzmsg << "GazeboUnderwater: reference link not defined, using instead: " << link->GetName() << endl;
            return link;
        }
    }

    double GazeboUnderwater::computeModelMass(physics::ModelPtr model) const
    {
        double mass = 0;
        physics::Link_V links = model->GetLinks();
        for (physics::Link_V::iterator it = links.begin(); it != links.end(); ++it)
            mass += (*it)->GetInertial()->GetMass();
        return mass;
    }

    void GazeboUnderwater::loadParameters(void)
    {
        waterLevel = getParameter<double>("water_level","meters", 0.0);
        fluidVelocity = getParameter<math::Vector3>("fluid_velocity","m/s",math::Vector3(0,0,0));
        linearDampCoefficients = getParameter<math::Vector3>("linear_damp_coefficients","N.s/m",
                math::Vector3(50,50,50));
        linearDampAngleCoefficients = getParameter<math::Vector3>("linear_damp_angle_coefficients","N.s",
                math::Vector3(45,45,45));
        quadraticDampCoefficients = getParameter<math::Vector3>("quadratic_damp_coefficients","N.s2/m2",
                math::Vector3(40,40,40));
        quadraticDampAngleCoefficients = getParameter<math::Vector3>("quadratic_damp_angle_coefficients","N.s2/m",
                math::Vector3(35,35,35));
        // buoyancy must be the difference between the buoyancy when the model is completely submersed and the model weight
        buoyancy = getParameter<double>("buoyancy","N", 5);
        buoyancy = abs(buoyancy) + computeModelMass(model) * world->GetPhysicsEngine()->GetGravity().GetLength();
        centerOfBuoyancy = getParameter<math::Vector3>("center_of_buoyancy","meters",
                math::Vector3(0, 0, 0.15));
    }

    void GazeboUnderwater::updateBegin(common::UpdateInfo const& info)
    {
        applyBuoyancy();
        applyViscousDamp();
    }

    void GazeboUnderwater::applyViscousDamp()
    {
        math::Vector3 cogPosition = link->GetWorldCoGPose().pos;
        double distanceToSurface = waterLevel - cogPosition.z;
        if( distanceToSurface > 0 )
        {
            // Calculates the difference between the model and fluid velocity relative to the world frame
            math::Vector3 velocityDifference = link->GetWorldLinearVel() - fluidVelocity;
            math::Vector3 angularVelocity = link->GetWorldAngularVel();

            // Get model quaternion and rotate velocity difference to get the velocity relative to the
            // model frame. This velocity is necessary to calculate the drag forces.
            // The drag forces have to be transformed to the world frame (with RotateVectorReverse),
            // before being applied to the model.
            // mf = model frame
            math::Quaternion modelQuaternion = link->GetWorldCoGPose().rot;
            math::Vector3 mfVelocity = modelQuaternion.RotateVector( velocityDifference );
            math::Vector3 mfAngularVelocity = modelQuaternion.RotateVector( angularVelocity );

            // Linear damp
            math::Vector3 linearDamp = - linearDampCoefficients * mfVelocity;
            link->AddForceAtWorldPosition(modelQuaternion.RotateVectorReverse(linearDamp),cogPosition);

            math::Vector3 linearAngularDamp = - linearDampAngleCoefficients * mfAngularVelocity;
            link->AddTorque( modelQuaternion.RotateVectorReverse(linearAngularDamp) );

            // Quadratic damp
            math::Vector3 quadraticDamp = - quadraticDampCoefficients *
                     mfVelocity.GetAbs() * mfVelocity;
            link->AddForceAtWorldPosition(modelQuaternion.RotateVectorReverse(quadraticDamp),cogPosition);

            math::Vector3 quadraticAngularDamp = - quadraticDampAngleCoefficients *
                     mfAngularVelocity.GetAbs() * mfAngularVelocity;
            link->AddTorque( modelQuaternion.RotateVectorReverse(quadraticAngularDamp) );
        }
    }

    void GazeboUnderwater::applyBuoyancy()
    {
        math::Vector3 cogPosition = link->GetWorldCoGPose().pos;
        double distanceToSurface = waterLevel - cogPosition.z;
        math::Vector3 linkBuoyancy;

        // The buoyancy is proportional no the submersed volume
        double submersedRatio = calculateSubmersedRatio(distanceToSurface);
        linkBuoyancy = math::Vector3(0,0,submersedRatio * buoyancy );

        math::Vector3 cobPosition = link->GetWorldCoGPose().pos +
                link->GetWorldCoGPose().rot.RotateVector(centerOfBuoyancy);
        link->AddForceAtWorldPosition(linkBuoyancy,cobPosition);
    }

    double GazeboUnderwater::calculateSubmersedRatio(double distanceToSurface) const
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
        return submersedVolume/linkVolume;
    }
}
