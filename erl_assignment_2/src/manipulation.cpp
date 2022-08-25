#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


/**
 * @brief   our move group interface
 */
moveit::planning_interface::MoveGroupInterface *move_group_interface;

/**
 * @brief   manupulation Service server 
 */
ros::ServiceServer srv_manip;



/**
 * \brief   move the robot in one pose
 *
 * \details This function allows the robot to move its arm in a certain 
 *          Pose, depending on whether gather_hint gets true or 
 *          false
 *
 * \note    The "home" Pose, refers to the partially extended, initial 
 *          Pose of the detectiBot's arm. Otherwise, the "gather_hint"
 *          refers to the pose needed for gathering the hint, nearby 
 *          the marker 
 *
 * \param[in]     gather_hint    Boolean variable needed for distinguishing
 *                               between two differet detectibot's arm Poses 
 * 
 * 
 *
 * \return  void
 */
void move_arm(bool gather_hint)
{
	if(gather_hint)
	{
		// gather_hint
		/**
 		* @brief   manipulation Service server 
 		*/
		move_group_interface->setNamedTarget( "gather_hint" );
	}
	else
	{
		// home
		move_group_interface->setNamedTarget( "home" );
	}
	
	move_group_interface->move( );
}



/**
 * \brief   manipulation Service callback implementatiomn 
 *
 * \details This function represents the callback of the maipulaiton
 *          service
 *
 * \note    It employs the move_arm funciton for letting Detectibot 
 *          move its arm between the initial arm pose (namely "home")
 *          or allowing for the hint collection
 *
 * \param[in]     req   A request of type std_srvs::SetBool,
 *                      enabling/disabling certain behaviours 
 * \param[in]     res   A response of type std_srvs::SetBool,
 *                      indicating a successfull run of triggered 
 *                      service 
 *                      
 * 
 *
 * \return  void
 */
bool cbk_manip(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
	move_arm(req.data);
	
	res.success = true;
	return true;
}


/// main function
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "manipulation");
	ros::NodeHandle nh;
	
	ros::AsyncSpinner spinner(2);
	spinner.start( );
	
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface move_group_interfacei( PLANNING_GROUP );
	move_group_interface = &move_group_interfacei;
	move_group_interface->setPlanningTime(10.0);
	
	
	/**
	* @brief   before starting, move the robot in the init pose
	*/
	move_arm(false);
	
	srv_manip = nh.advertiseService("/manipulation", cbk_manip);
	
	ros::waitForShutdown( );
	
	return 0;
}
