#include "GazeboUnderwater.hpp"
#include <gazebo/common/Exception.hh>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>
#include <stdlib.h>

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
        std::string damp_matrices;
        damp_matrices += "50 0 0 0 0 0\n";
        damp_matrices += "0 50 0 0 0 0\n";
        damp_matrices += "0 0 50 0 0 0\n";
        damp_matrices += "0 0 0 45 0 0\n";
        damp_matrices += "0 0 0 0 45 0\n";
        damp_matrices += "0 0 0 0 0 45\n\n";
        damp_matrices += "40 0 0 0 0 0\n";
        damp_matrices += "0 40 0 0 0 0\n";
        damp_matrices += "0 0 40 0 0 0\n";
        damp_matrices += "0 0 0 35 0 0\n";
        damp_matrices += "0 0 0 0 35 0\n";
        damp_matrices += "0 0 0 0 0 35";
        dampingCoefficients = convertToMatrices(getParameter<string>("damping_coefficients","N/vel^n / vel^n={m/s, rad/s, m2/s2. rad2/s2}",damp_matrices));

        std::string extra_inertia;
        extra_inertia += "0 0 0 0 0 0\n";
        extra_inertia += "0 0 0 0 0 0\n";
        extra_inertia += "0 0 0 0 0 0\n";
        extra_inertia += "0 0 0 0 0 0\n";
        extra_inertia += "0 0 0 0 0 0\n";
        extra_inertia += "0 0 0 0 0 0";
        addedInertia = convertToMatrix(getParameter<string>("added_inertia","Kg, Kg.m2", extra_inertia));


    }

    void GazeboUnderwater::updateBegin(common::UpdateInfo const& info)
    {
        applyBuoyancy();
        applyDamp();
    }

    void GazeboUnderwater::applyDamp()
    {
        math::Vector3 cogPosition = link->GetWorldCoGPose().pos;
        double distanceToSurface = waterLevel - cogPosition.z;
        // Define damping as proportional to the submersed volume
        double submersedRatio = calculateSubmersedRatio(distanceToSurface);

        if( distanceToSurface > 0 )
        {
            math::Quaternion modelQuaternion = link->GetWorldCoGPose().rot;
            base::Vector6d velocities = getModelFrameVelocities();

            base::Vector6d damp = base::Vector6d::Zero();
            if(dampingCoefficients.size() == 2)
              damp = - dampingCoefficients[0] * velocities - dampingCoefficients[1] * velocities.cwiseAbs().asDiagonal() * velocities;
            else if(dampingCoefficients.size() == 6)
            {
              base::Matrix6d dampMatrix = base::Matrix6d::Zero();
              for(size_t i=0; i < dampingCoefficients.size(); i++)
                  dampMatrix += dampingCoefficients[i] * velocities.cwiseAbs()[i];
              damp = - dampMatrix * velocities;
            }
            else
              gzthrow("GazeboUnderwater: Damping Parameter has wrong dimension!");
            // Define damping as proportional to the submersed volume
            damp *= submersedRatio;

            math::Vector3 linearDamp(damp[0], damp[1], damp[2]);
            math::Vector3 angularDamp(damp[3], damp[4], damp[5]);

            link->AddForceAtWorldPosition(modelQuaternion.RotateVectorReverse(linearDamp),cogPosition);
            link->AddTorque( modelQuaternion.RotateVectorReverse(angularDamp) );
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

    void GazeboUnderwater::applyCoriolisAddedInertia()
    {
        /**
         * Based on McFarland[2013] and Fossen[1994]
         * coriolisEffect = H(M*v)*v
         * M = inertiaMatrix; v = velocity
         * Operator H: R^6 -> R^(6x6).
         *      H(v) = [0(3x3), J(v.head(3));
         *              J(v.head(3)),  J(v.tail(3))]
         * Operator J: R^3 -> R^(3x3) (the so(3) operator, skew-symmetric matrix)
         *      J([v1; v2; v3]) = [ 0 ,-v3, v2;
         *                          v3, 0 ,-v1;
         *                         -v2, v1, 0]
         * Cross product:
         *      J(v.head(3)) * v.tail(3) = v.head(3) X v.tail(3)
         */
        base::Vector6d velocities = getModelFrameVelocities();
        base::Vector6d coriloisEffect;
        base::Vector6d prod = addedInertia * velocities;
        coriloisEffect << prod.head<3>().cross(velocities.tail<3>()),
                    prod.head<3>().cross(velocities.head<3>()) + prod.tail<3>().cross(velocities.tail<3>());

        math::Vector3 linearCoriolis(-coriloisEffect[0], -coriloisEffect[1], -coriloisEffect[2]);
        math::Vector3 angularCoriolis(-coriloisEffect[3], -coriloisEffect[4], -coriloisEffect[5]);

        math::Vector3 cogPosition = link->GetWorldCoGPose().pos;
        math::Quaternion modelQuaternion = link->GetWorldCoGPose().rot;
        link->AddForceAtWorldPosition(modelQuaternion.RotateVectorReverse(linearCoriolis),cogPosition);
        link->AddTorque( modelQuaternion.RotateVectorReverse(angularCoriolis) );
    }

    std::vector<base::Matrix6d> GazeboUnderwater::convertToMatrices(const std::string &matrices)
    {
        std::vector<base::Matrix6d> ret;
        std::vector<std::string> splitted;
        boost::algorithm::split_regex( splitted, matrices, boost::regex( "\n\n" ));
        for( size_t i=0; i<splitted.size(); i++)
            std::cout << "damping: " << splitted.at(i) << std::endl;
        if (splitted.size() != 2 && splitted.size() != 6)
            gzthrow("GazeboUnderwater: Damping Parameters has not 2 or 6 matrices!");
        for( size_t i=0; i<splitted.size(); i++)
            ret.push_back(convertToMatrix(splitted.at(i)));
        return ret;
    }

    base::Matrix6d GazeboUnderwater::convertToMatrix(const std::string &matrix)
    {
        base::Matrix6d ret;
        std::vector<std::string> splitted;
        boost::split( splitted, matrix, boost::is_any_of( "\n" ), boost::token_compress_on );
        if (splitted.size() != 6)
            gzthrow("GazeboUnderwater: Matrix has not 6 lines!");
        for( size_t i=0; i<6; i++)
        {
            std::vector<std::string> line;
            boost::split( line, splitted[i], boost::is_any_of( " " ), boost::token_compress_on );
            if (line.size() != 6)
                gzthrow("GazeboUnderwater: Line has not 6 columns!");
            for(size_t j=0; j<6; j++)
                ret(i,j) = atof(line.at(j).c_str());
        }
        return ret;
    }

    base::Vector6d GazeboUnderwater::getModelFrameVelocities()
    {
        // Calculates the difference between the model and fluid velocity relative to the world frame
        math::Vector3 velocityDifference = link->GetWorldLinearVel() - fluidVelocity;
        math::Vector3 angularVelocity = link->GetWorldAngularVel();

        math::Quaternion modelQuaternion = link->GetWorldCoGPose().rot;
        math::Vector3 mfVelocity = modelQuaternion.RotateVector( velocityDifference );
        math::Vector3 mfAngularVelocity = modelQuaternion.RotateVector( angularVelocity );
        base::Vector6d velocities = base::Vector6d::Zero();
        velocities << mfVelocity[0], mfVelocity[1], mfVelocity[2], mfAngularVelocity[0], mfAngularVelocity[1], mfAngularVelocity[2];
        return velocities;
    }
}
