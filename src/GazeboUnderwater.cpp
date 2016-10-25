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

    physics::Inertial GazeboUnderwater::computeModelInertial(physics::ModelPtr model) const
    {
        Inertial inertial(0);
        inertial.SetMOI(math::Matrix3::ZERO);
        Inertial temp;
        physics::Link_V links = model->GetLinks();
        for (physics::Link_V::iterator it = links.begin(); it != links.end(); ++it)
            if(!(*it)->GetKinematic())
            {
                // Set Inertial's CoG related with the parent link
                temp = *(*it)->GetInertial();
                math::Pose pose = (*it)->GetRelativePose();
                pose.pos += pose.rot.RotateVector(temp.GetCoG());
                temp.SetCoG(pose);
                inertial += temp;
            }
        return inertial;
    }

    void GazeboUnderwater::loadParameters(void)
    {
        waterLevel = getParameter<double>("water_level","meters", 0.0);
        fluidVelocity = getParameter<math::Vector3>("fluid_velocity","m/s",math::Vector3(0,0,0));
        modelInertial = computeModelInertial(model);

        // buoyancy must be the difference between the buoyancy when the model is completely submersed and the model weight
        buoyancy = getParameter<double>("buoyancy","N", 5);
        buoyancy = abs(buoyancy) + modelInertial.GetMass() * world->GetPhysicsEngine()->GetGravity().GetLength();
        // centerOfBuoyancy must be positioned related to the model's center of gravity.
        centerOfBuoyancy = getParameter<math::Vector3>("center_of_buoyancy","meters",
                math::Vector3(0, 0, 0.15));
        centerOfBuoyancy += modelInertial.GetCoG();
        std::string damp_matrices;
        damp_matrices += "50 0 0 0 0 0;";
        damp_matrices += "0 50 0 0 0 0;";
        damp_matrices += "0 0 50 0 0 0;";
        damp_matrices += "0 0 0 45 0 0;";
        damp_matrices += "0 0 0 0 45 0;";
        damp_matrices += "0 0 0 0 0 45\n";
        damp_matrices += "40 0 0 0 0 0;";
        damp_matrices += "0 40 0 0 0 0;";
        damp_matrices += "0 0 40 0 0 0;";
        damp_matrices += "0 0 0 35 0 0;";
        damp_matrices += "0 0 0 0 35 0;";
        damp_matrices += "0 0 0 0 0 35";
        dampingCoefficients = convertToMatrices(getParameter<string>("damping_coefficients","N/vel^n / vel^n={m/s, rad/s, m2/s2. rad2/s2}",damp_matrices));

        std::string extra_inertia;
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0;";
        extra_inertia += "0 0 0 0 0 0";
        addedInertia = convertToMatrix(getParameter<string>("added_inertia","Kg, Kg.m2", extra_inertia));

        Matrix6 gzInertia;
        gzInertia.top_left = modelInertial.GetMass() * math::Matrix3::IDENTITY;
        gzInertia.bottom_right = modelInertial.GetMOI();
        Matrix6 sum_inertia = gzInertia + addedInertia;
        std::cout<<"inertia top_left: " << sum_inertia.top_left << std::endl << std::endl;
        std::cout<<"inertia top_right: " << sum_inertia.top_right << std::endl << std::endl;
        std::cout<<"inertia bottom_left: " << sum_inertia.bottom_left << std::endl << std::endl;
        std::cout<<"inertia bottom_right: " << sum_inertia.bottom_right << std::endl << std::endl;

        for (int i=0; i<dampingCoefficients.size(); i++)
        {   std::cout <<"Damping["<<i<<"]" <<std::endl;
            std::cout<<"damp["<<i<<"] top_left: "<< dampingCoefficients[i].top_left << std::endl << std::endl;
            std::cout<<"damp["<<i<<"] top_right: "<< dampingCoefficients[i].top_right << std::endl << std::endl;
            std::cout<<"damp["<<i<<"] bottom_left: "<< dampingCoefficients[i].bottom_left << std::endl << std::endl;
            std::cout<<"damp["<<i<<"] bottom_right: "<< dampingCoefficients[i].bottom_right << std::endl << std::endl;
        }

        std::cout << "weight: "<< modelInertial.GetMass() * world->GetPhysicsEngine()->GetGravity().GetLength() << std::endl;
        std::cout << "buoy: "<< buoyancy << std::endl;
        std::cout << "CoG: "<< modelInertial.GetCoG() << std::endl;

        Matrix6 identity(math::Matrix3::IDENTITY, math::Matrix3::ZERO, math::Matrix3::ZERO, math::Matrix3::IDENTITY);

        // gzInertia should be inversible
        compensatedInertia = gzInertia * sum_inertia.Inverse() - identity;

        previousCompensatedEffort.top = math::Vector3::Zero;
        previousCompensatedEffort.bottom = math::Vector3::Zero;
    }

    void GazeboUnderwater::updateBegin(common::UpdateInfo const& info)
    {
        applyBuoyancy();
        applyDamp();
        applyCoriolisAddedInertia();
        applyCompensatedEffort();
    }

    void GazeboUnderwater::applyDamp()
    {
        double distanceToSurface = waterLevel - link->GetWorldPose().pos.z;

        if( distanceToSurface > 0 )
        {
            Vector6 vel = getModelFrameVelocities();

            Vector6 damp;
            if(dampingCoefficients.size() == 2)
            {
                Vector6 vel_square(vel.top*vel.top.GetAbs(), vel.bottom*vel.bottom.GetAbs());
                damp = dampingCoefficients[0] * vel + dampingCoefficients[1] * vel_square;
            }
            else if(dampingCoefficients.size() == 6)
            {
                Matrix6 dampMatrix;
                Vector6 vel_abs(vel.top.GetAbs(), vel.bottom.GetAbs());
                for(size_t i=0; i < 3; i++)
                {
                    dampMatrix += dampingCoefficients[i] * vel_abs.top[i];
                    dampMatrix += dampingCoefficients[i+3] * vel_abs.bottom[i];
                }
                damp = dampMatrix * vel;
            }
            else
              gzthrow("GazeboUnderwater: Damping Parameter has wrong dimension!");
            // Define damping as proportional to the submerged volume.
            // Add minus for indicate resistence efforts.
            damp *= -calculateSubmersedRatio(distanceToSurface);

            link->AddLinkForce(damp.top, modelInertial.GetCoG());
            link->AddRelativeTorque(damp.bottom);
           }
       }

    void GazeboUnderwater::applyBuoyancy()
    {
        double distanceToSurface = waterLevel - link->GetWorldPose().pos.z;
        // The buoyancy is proportional no the submersed volume
        double submersedRatio = calculateSubmersedRatio(distanceToSurface);

        math::Vector3 modelBuoyancy = math::Vector3(0,0,submersedRatio * buoyancy );
        math::Vector3 cobPosition = link->GetWorldPose().pos +
                link->GetWorldPose().rot.RotateVector(centerOfBuoyancy);

        link->AddForceAtWorldPosition(modelBuoyancy,cobPosition);
    }

    double GazeboUnderwater::calculateSubmersedRatio(double distanceToSurface) const
    {
        math::Box linkBoudingBox = link->GetBoundingBox();
        double linkVolume = linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength() * linkBoudingBox.GetZLength();
        double submersedVolume = 0.0;

        if(distanceToSurface <= 0.0)
            submersedVolume = 0.0;
        else{
            if(distanceToSurface <= (linkBoudingBox.GetZLength())/2.0 )
                submersedVolume = linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength() *
                        ( linkBoudingBox.GetZLength()/2.0 + distanceToSurface );
            else
                submersedVolume = linkBoudingBox.GetXLength() * linkBoudingBox.GetYLength() *
                        linkBoudingBox.GetZLength();
        }
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
        Vector6 velocities = getModelFrameVelocities();
        Vector6 momentum = addedInertia * velocities;
        Vector6 coriloisEffect( momentum.top.Cross(velocities.bottom),
                    momentum.top.Cross(velocities.top) + momentum.bottom.Cross(velocities.bottom));

        link->AddLinkForce(coriloisEffect.top, modelInertial.GetCoG());
        link->AddRelativeTorque(coriloisEffect.bottom);
    }

    void GazeboUnderwater::applyCompensatedEffort()
    {
        /**
        * AUV acceleration
        *  acceleration = (M+Ma)^-1 * (Thruster - Coriolis - Coriolis_added_mass - Buoyancy - Damping)
        *  acceleration = (M+Ma)^-1 * F; F = (Thruster - Coriolis - Coriolis_added_mass - Buoyancy - Damping)
        *
        * Acceleration computed by Gazebo
        *  acceleration' = M^-1 * F'
        *
        * acceleration = acceleration'
        * (M+Ma)^-1 * F = M^-1 * F'
        * F' = F + C  => (M+Ma)^-1 * F = M^-1 * (F + C)
        * C = (M*(M+Ma)^-1 - I) * F; F = GetForceTorque()??
        *
        * Compesanted efforts C will make Gazebo compute the expected acceleration, ignoring the added mass matrix
        */
        // Force in link's CoG. The same force is at the model's CoG, plus a torque
        // math::Vector3 force = link->GetWorldPose().rot.RotateVectorReverse(link->GetWorldForce());
        // math::Vector3 torque = link->GetWorldPose().rot.RotateVectorReverse(link->GetWorldTorque())
        //         + force.Cross(modelInertial.GetCoG() - link->GetInertial()->GetCoG());

        math::Vector3 force = link->GetRelativeForce();
        math::Vector3 torque = link->GetRelativeTorque()
            + (link->GetInertial()->GetCoG() - modelInertial.GetCoG()).Cross(force);

        Vector6 efforts(force, torque);
        Vector6 compEfforts = compensatedInertia * (efforts - previousCompensatedEffort);

        previousCompensatedEffort = compEfforts;

        link->AddLinkForce(compEfforts.top, modelInertial.GetCoG());
        link->AddRelativeTorque(compEfforts.bottom);
    }

    std::vector<Matrix6> GazeboUnderwater::convertToMatrices(const std::string &matrices)
    {
        std::vector<Matrix6> ret;
        std::vector<std::string> splitted;
        boost::algorithm::split_regex( splitted, matrices, boost::regex( "\n" ));
        std::cout << "/* matrices */ "<< splitted.size()<< std::endl;
        if (splitted.size() != 2 && splitted.size() != 6)
            gzthrow("GazeboUnderwater: Damping Parameters has not 2 or 6 matrices!");
        for( size_t i=0; i<splitted.size(); i++)
            ret.push_back(convertToMatrix(splitted.at(i)));
        return ret;
    }

    Matrix6 GazeboUnderwater::convertToMatrix(const std::string &matrix)
    {
        Matrix6 ret;
        std::vector<std::string> splitted;
        boost::split( splitted, matrix, boost::is_any_of( ";" ), boost::token_compress_on );
        std::cout << "/* matrix lines */ "<< splitted.size()<< std::endl;
        if (splitted.size() != 6)
            gzthrow("GazeboUnderwater: Matrix has not 6 lines!");
        for( size_t i=0; i<6; i++)
        {
            std::vector<std::string> line;
            boost::split( line, splitted[i], boost::is_any_of( " " ), boost::token_compress_on );
            std::cout << "/* matrix columns */ "<< line.size()<< std::endl;
            if (line.size() != 6)
                gzthrow("GazeboUnderwater: Line has not 6 columns!" + splitted.size());
            for(size_t j=0; j<6; j++)
            {
                if(i<3 && j<3)
                    ret.top_left[i][j] = atof(line.at(j).c_str());
                else if(i<3 && j>=3)
                    ret.top_right[i][j-3] = atof(line.at(j).c_str());
                else if(i>=3 && j<3)
                    ret.bottom_left[i-3][j] = atof(line.at(j).c_str());
                else if(i>=3 && j>=3)
                    ret.bottom_right[i-3][j-3] = atof(line.at(j).c_str());
            }
        }
        return ret;
    }

    Vector6 GazeboUnderwater::getModelFrameVelocities()
    {
        Vector6 velocities;
        // Calculates the difference between the model and fluid velocity relative to the world frame
        math::Vector3 velocityDifference = link->GetWorldLinearVel(modelInertial.GetCoG()) - fluidVelocity;
        velocities.top = link->GetWorldPose().rot.RotateVectorReverse( velocityDifference );
        velocities.bottom = link->GetRelativeAngularVel();
        return velocities;
    }

}
