#ifndef _GAZEBOUNDERWATER_HPP_
#define _GAZEBOUNDERWATER_HPP_

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo_underwater
{
	class GazeboUnderwater : public gazebo::WorldPlugin
	{
	    private: 
	        void updateBegin(gazebo::common::UpdateInfo const& info); 
	        void applyBuoyancy();
	        void loadParameters(); 
	        template <class T> T getParameter(std::string); 

            gazebo::physics::WorldPtr world;
	        gazebo::physics::ModelPtr model; 

            sdf::ElementPtr sdf;
	        std::vector<gazebo::event::ConnectionPtr> eventHandler;
	        double length,width,height;  
            double water_level;       // dimension in meter
	    	double liquid_weight;     // 1 liter of water = 1 dm^3 =~ 1 kg.
    		double viscous_damping;
		
		public: 
		    virtual void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
            
	};
	
	GZ_REGISTER_WORLD_PLUGIN(GazeboUnderwater)
} // end namespace gazebo_underwater

#endif
