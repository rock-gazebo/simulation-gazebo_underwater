#ifndef _GAZEBOUNDERWATER_HPP_
#define _GAZEBOUNDERWATER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/Vector3.hh>

namespace gazebo_underwater
{
    class GazeboUnderwater : public gazebo::WorldPlugin
    {
        private: 
            void updateBegin(gazebo::common::UpdateInfo const& info); 
            void applyBuoyancy();
            void applyViscousDamp();
            void loadParameters();
            template <typename T>
            T getParameter(std::string parameter_name, std::string dimension, T default_value);
            double calculateSubmersedVolume(double);

            gazebo::physics::WorldPtr world;
            gazebo::physics::ModelPtr model;
            gazebo::physics::LinkPtr link;

            sdf::ElementPtr sdf;
            std::vector<gazebo::event::ConnectionPtr> eventHandler;

            gazebo::math::Vector3 centerOfBuoyancy;
            gazebo::math::Vector3 fluidVelocity;
            gazebo::math::Vector3 linearDampCoefficients;
            gazebo::math::Vector3 linearDampAngleCoefficients;
            gazebo::math::Vector3 quadraticDampCoefficients;
            gazebo::math::Vector3 quadraticDampAngleCoefficients;
            gazebo::math::Vector3 sideAreas;
            double volume;
            double waterLevel;       // dimension in meter
            double densityOfFluid;
            double buoyancy;

        public:
            GazeboUnderwater();
            ~GazeboUnderwater();
            virtual void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    };

    GZ_REGISTER_WORLD_PLUGIN(GazeboUnderwater)
} // end namespace gazebo_underwater

#endif
