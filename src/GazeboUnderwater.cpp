#include "GazeboUnderwater.hpp"
#include "gazebo/math/Vector3.hh"


namespace gazebo_underwater
{
    void GazeboUnderwater::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        gzmsg << "GazeboUnderwater: Loading underwater environment." << std::endl;
        
        world = _world; 
        sdf = _sdf; 
        
        loadParameters(); 
        
	    // Each simulation step the Update method is called to update the simulated sensors and actuators
	    eventHandler.push_back(
			    gazebo::event::Events::ConnectWorldUpdateBegin(
					    boost::bind(&GazeboUnderwater::updateBegin,this, _1)));
    }


    template <class T> 
    T GazeboUnderwater::getParameter(std::string _parameter_name)
    {
        T var;
        if(sdf->HasElement(_parameter_name.c_str()))
        {
            var = sdf->Get< T >(_parameter_name.c_str());
            gzmsg << "GazeboUnderwater: " + _parameter_name + ": " << var << std::endl; 
        }else{
            gzmsg << "GazeboUnderwater: " + _parameter_name + " not found. Quit simulation... " << std::endl;
            gazebo::shutdown();
        }        
        return var;
    }
            

    void GazeboUnderwater::loadParameters(void)
    {
        // If these parameters are not found the simulation must stop. 
        if(sdf->HasElement("model_name"))   
        {
            model = world->GetModel( sdf->Get<std::string>("model_name") );  
            gzmsg << "GazeboUnderwater: found model:" << model->GetName() << std::endl;
        }else{
            gzmsg << "GazeboUnderwater: model not found. Quit simulation... " << std::endl;
        }
        
        width = getParameter<double>("width");
        length = getParameter<double>("length");
        height = getParameter<double>("height");
        water_level = getParameter<double>("water_level");
        viscous_damping = getParameter<double>("viscous_damping");
        liquid_weight = getParameter<double>("liquid_weight");
    }

    void GazeboUnderwater::updateBegin(gazebo::common::UpdateInfo const& info)
    {
        applyBuoyancy(); 
    }


    void GazeboUnderwater::applyBuoyancy()
    {
    	// Apply buoyancy to links in underwater environment
		gazebo::math::Vector3 cobPosition;
		gazebo::math::Vector3 velocityDifference;
		gazebo::math::Vector3 gravity = world->GetPhysicsEngine()->GetGravity();
		gazebo::math::Vector3 link_buoyancy;
		double distancetosurface = 0.0; // Distance to surface is given in meters
		double relative_height = 0.0;   // dimension in meter
		double submersed_volume = 0.0;  // it must be dm^3
		double compensation = 1.0;
		gazebo::math::Vector3 buoyancy_center(0.0, 0.0, 0.0);
//	    gazebo::math::Vector3 viscous_damping(0.5,0.5,0.5); 
		gazebo::math::Vector3 fluid_velocity(0.0, 0.0, 0.0);
//		gazebo::math::Vector3 link_friction(0.0, 0.0, 0.0);
//		gazebo::math::Vector3 resultant_force(0.0, 0.0, 0.0);
	
	    gazebo::physics::LinkPtr link = model->GetLink( sdf->Get<std::string>("link_name") ); 
   		
		cobPosition = link->GetWorldPose().pos + 
				link->GetWorldPose().rot.RotateVector(buoyancy_center);		
	
		// link_buoyancy goes to zero when the model link is above the surface. 
		// it depends on the volume submersed
		distancetosurface = water_level - cobPosition.z;
		if(distancetosurface <= 0 )
		{ 
			submersed_volume = 0;
		} else{
			if(distancetosurface <= height)  // distancetosurface is in meters
			{	
				relative_height = distancetosurface;
				submersed_volume = length * width * relative_height * 1000;  
			}else
			{
				submersed_volume = length * width * height * 1000;
			}
		}
		// The buoyancy opposes gravity	=> it is negative.		
		link_buoyancy = - compensation * submersed_volume * liquid_weight * gravity;
		
		// Calculates dynamic viscous damp
		velocityDifference = link->GetWorldPose().rot.RotateVectorReverse(link->GetWorldLinearVel() - fluid_velocity);
//		velocityDifference = link->GetWorldLinearVel() + fluid_velocity;	
		link_buoyancy -= link->GetWorldPose().rot.RotateVector(
					viscous_damping * velocityDifference) ;
//		link_friction = - link->GetWorldPose().rot.RotateVector( math::Vector3(
//						  viscous_damping.x * velocityDifference.x,
//						  viscous_damping.y * velocityDifference.y,
//						  viscous_damping.z * velocityDifference.z));			
		
		
		// Gazebo adds the link weight, so there is no need to calculate it. 
		// resultant_force = link_buoyancy + link_friction; 
	
		link->AddForceAtWorldPosition(link_buoyancy, cobPosition);											
//		link->AddForceAtWorldPosition(resultant_force, cobPosition);
//  	link->AddForce(link_buoyancy);

    }   // applyBuoyancy() end
}   // gazebo_underwater namespace


