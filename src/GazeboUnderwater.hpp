#ifndef _GAZEBOUNDERWATER_HPP_
#define _GAZEBOUNDERWATER_HPP_

#include <gazebo/gazebo.hh>
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
            void applyViscousFriction();
            void loadParameters();
            template <typename T>
            T getParameter(std::string _parameter_name, std::string dimension, T default_value);
            double calculateSubmersedVolume(double,double,double,double);

            gazebo::physics::WorldPtr world;
            gazebo::physics::ModelPtr model;
            gazebo::physics::LinkPtr link;

            sdf::ElementPtr sdf;
            std::vector<gazebo::event::ConnectionPtr> eventHandler;

            gazebo::math::Vector3 size;
            gazebo::math::Vector3 centerOfBuoyancy;
            gazebo::math::Vector3 fluidVelocity;
            gazebo::math::Vector3 viscousDamping;
            double waterLevel;       // dimension in meter
            double densityOfFluid;
            double buoyancy;

        public: 
            virtual void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    };

    GZ_REGISTER_WORLD_PLUGIN(GazeboUnderwater)
} // end namespace gazebo_underwater

#endif
