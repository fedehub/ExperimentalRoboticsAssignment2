
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


/// move group interface
moveit::planning_interface::MoveGroupInterface *mgi;

/// client /manipulation
ros::ServiceServer srv_manip;


/// move the robot in one pose
void move_arm( bool gather_hint )
{
	if( gather_hint )
	{
		// gather_hint
		mgi->setNamedTarget( "gather_hint" );
	}
	else
	{
		// home
		mgi->setNamedTarget( "home" );
	}
	
	mgi->move( );
}


/// implementation of the service /manipulation
bool cbk_manip( std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res )
{
	move_arm( req.data );
	
	res.success = true;
	return true;
}


/// node main function
int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "manipulation" );
	ros::NodeHandle nh;
	
	ros::AsyncSpinner spinner(2);
	spinner.start( );
	
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface mgii( PLANNING_GROUP );
	mgi = &mgii;
	mgi->setPlanningTime(10.0);
	
	// before starting, move the robot in the init pose
	move_arm( false );
	
	srv_manip = nh.advertiseService( "/manipulation", cbk_manip );
	
	ros::waitForShutdown( );
	
	return 0;
}
