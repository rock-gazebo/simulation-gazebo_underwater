#ifndef _GAZEBOUNDERWATER_HPP_
#define _GAZEBOUNDERWATER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/Vector3.hh>

namespace gazebo_underwater
{
    class GazeboUnderwater : public gazebo::WorldPlugin
    {
            typedef gazebo::physics::ModelPtr ModelPtr;
            typedef gazebo::physics::WorldPtr WorldPtr;
            typedef gazebo::physics::LinkPtr LinkPtr;
            typedef gazebo::physics::Inertial Inertial;

        private: 
            void updateBegin(gazebo::common::UpdateInfo const& info); 
            void applyBuoyancy();
            void applyViscousDamp();
            ModelPtr getModel(WorldPtr world, sdf::ElementPtr sdf) const;
            LinkPtr getReferenceLink(ModelPtr model, sdf::ElementPtr sdf) const;
            void loadParameters();
            template <typename T>
            T getParameter(std::string parameter_name, std::string dimension, T default_value) const;
            double calculateSubmersedRatio(double) const;

            WorldPtr world;
            ModelPtr model;
            LinkPtr link;

            sdf::ElementPtr sdf;
            std::vector<gazebo::event::ConnectionPtr> eventHandler;

            gazebo::math::Vector3 centerOfBuoyancy;
            gazebo::math::Vector3 fluidVelocity;
            gazebo::math::Vector3 linearDampCoefficients;
            gazebo::math::Vector3 linearDampAngleCoefficients;
            gazebo::math::Vector3 quadraticDampCoefficients;
            gazebo::math::Vector3 quadraticDampAngleCoefficients;
            double waterLevel;       // dimension in meter
            double buoyancy;

        public:
            GazeboUnderwater();
            ~GazeboUnderwater();
            virtual void Load(WorldPtr _parent, sdf::ElementPtr _sdf);
    };

    GZ_REGISTER_WORLD_PLUGIN(GazeboUnderwater)
} // end namespace gazebo_underwater

#endif
