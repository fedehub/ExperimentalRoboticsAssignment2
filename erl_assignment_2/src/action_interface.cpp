
/** @ package erl_assignment_2
* 
*	@file action_interface.cpp
*	@brief This node implements all the rosplan actions in a single node 
*
*	@author Federico Civetta
*	@version 1.0.0
*   
*	Subscribes to: <BR>
*		/rosplan_plan_dispatcher/action_dispatch 		[rosplan_dispatch_msgs/ActionDispatch]
*
*	Publishes to: <BR>
*		/rosplan_knowledge_base/pddl_action_parameters 	[rosplan_knowledge_msgs/DomainFormula]
* 		/rosplan_plan_dispatcher/action_feedback 		[rosplan_dispatch_msgs/ActionFeedback]
*
*	Services: <BR>
*   	None
* 
*	Client Services: <BR>
*   	/go_to_point		[std_srvs/SetBool]
*    	/manipulation		[std_srvs/SetBool]
*		/get_id				[erl_assignment_2_msgs/GetId]
*		/oracle_solution	[erl2/Oracle]
*		/mark_wrong_id		[erl_assignment_2_msgs/MarkWrongId]
]
*
*	Action Services: <BR>
*    	None
*
*	Description: <BR>
*		This node implements the rosplan actions declared inside the 
*		pddl domain. For optimisation reasons, topics and services 
* 		gets allocated only once the action gets called for the first
*		time through the rosplan action dispatcher 
*
*
*
*/
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "rosplan_action_interface/RPActionInterface.h"
#include "diagnostic_msgs/KeyValue.h"
#include "erl_assignment_2_msgs/GetId.h"
#include "erl_assignment_2_msgs/MarkWrongId.h"
#include "erl2/Oracle.h"
#include "geometry_msgs/Point.h"

#include <string>
#include <vector>
#include <map>


/// (leave_temple ?from - temple ?to - waypoint)
#define LEAVE_TEMPLE "leave_temple"

/// (go_to_wp ?from ?to - waypoint)
#define GO_TO_WP "go_to_wp"

/// (shift_gripper ?wp - waypoint)
#define SHIFT_GRIPPER "shift_gripper"

/// (gather_hint ?wp - waypoint)
#define GATHER_HINT "gather_hint"

/// (reach_temple ?from - waypoint ?to - temple)
#define REACH_TEMPLE "reach_temple"

/// (check_consistent_hypo ?wp - waypoint)
#define CHECK_CONSISTENT_HYPO "check_consistent_hypo"

/// (query_hypo ?tp - temple)
#define QUERY_HYPO "query_hypo"


namespace KCL_rosplan 
{

/**  */
class action_interface_class : public RPActionInterface
{
public:
	/** class constructor */
	action_interface_class( ) : RPActionInterface( )
	{
		wps["center"] = geometry_msgs::Point( );
		wps["center"].x = 0.0;
		wps["center"].y = 0.0;
		
		wps["wp1"] = geometry_msgs::Point( );
		wps["wp1"].x = 2.75;
		wps["wp1"].y = 0.0;
		
		wps["wp2"] = geometry_msgs::Point( );
		wps["wp2"].x = 0.0;
		wps["wp2"].y = 2.75;
		
		wps["wp3"] = geometry_msgs::Point( );
		wps["wp3"].x = -2.75;
		wps["wp3"].y = 0.0;
		
		wps["wp4"] = geometry_msgs::Point( );
		wps["wp4"].x = 0.0;
		wps["wp4"].y = -2.75;
	}
	
	/** action callback */
	bool concreteCallback ( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		// check for the node setup
		if( !init_node )
		{
			action_name = msg->name;
			setup_node( );
			
			init_node = true;
		}
		
		// then, execute the action
		return exec_action( msg );
	}
	
	/** action initialiser dispatcher */
	void setup_node( )
	{
		if( action_name == LEAVE_TEMPLE )
			leave_temple_setup( );
		else if( action_name == GO_TO_WP )
			go_to_wp_setup( );
		else if( action_name == SHIFT_GRIPPER )
			shift_gripper_setup( );
		else if( action_name == GATHER_HINT )
			gather_hint_setup( );
		else if( action_name == REACH_TEMPLE )
			reach_temple_setup( );
		else if( action_name == CHECK_CONSISTENT_HYPO )
			check_consistent_hypo_setup( );
		else if( action_name == QUERY_HYPO )
			query_hypo_setup( );
		else
			ROS_WARN_STREAM( "unrecognised action -- action name: " << action_name );
	}
	
	/** action execution dispatcher */
	bool exec_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		if( action_name == LEAVE_TEMPLE )
			return leave_temple_action( msg );
		else if( action_name == GO_TO_WP )
			return go_to_wp_action( msg );
		else if( action_name == SHIFT_GRIPPER )
			return shift_gripper_action( msg );
		else if( action_name == GATHER_HINT )
			return gather_hint_action( msg );
		else if( action_name == REACH_TEMPLE )
			return reach_temple_action( msg );
		else if( action_name == CHECK_CONSISTENT_HYPO )
			return check_consistent_hypo_action( msg );
		else if( action_name == QUERY_HYPO )
			return query_hypo_action( msg );
		else
			ROS_WARN_STREAM( "unrecognised action -- action name: " << action_name );
		
		return false;
	}
	
private:
	
	/// node handle
	ros::NodeHandle nh;
	
	/// if the node has been initialised or not
	bool init_node = false;
	
	/// tne name of the action implemented by this node
	std::string action_name = "";
	
	/// go_to_point client
	ros::ServiceClient cl_nav;
	
	/// manipulation client
	ros::ServiceClient cl_manip;
	
	/// get_id client
	ros::ServiceClient cl_get_id;
	
	/// mark_wrong_id client
	ros::ServiceClient cl_mark;
	
	/// oracle solution client
	ros::ServiceClient cl_solution;
	
	/// waypoints
	std::map<std::string, geometry_msgs::Point> wps;
	
	
	
	
	// === leave_temple === // 
	

	/**
	 * \brief   setup action leave_temple
	 *
	 * \details This function allows the robot to leave the starting position
	 * 			to reach the following waypoint 
	 *
	 * \note    For "starting position" we refer to the starting position of 
	 * 			the robot (where it is spawned), coinciding with the temple
	 * 			location. 
	 *
	 *
	 * \return  void
	 */
	void leave_temple_setup( )
	{
		// (client) /go_to_point : std_srvs/SetBool
		cl_nav = nh.serviceClient<std_srvs::SetBool>( "/go_to_point" );
	}
	
	/** leave temple execution */
	bool leave_temple_action(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		ROS_INFO_STREAM(action_to_string( msg ));
		
		// target waypoint
		std::string wp_to = msg->parameters[0].value;
		
		//dal tempio, dirigiti verso il waypoint
		//    trova le coordinate corrispondenti al waypoint
		//    setta i parametri des_pos_x e des_pos_y con ros::param::set( "", val )
		//    e chiamata a servizio
		
		navigate_to( wp_to );
		
		return true;
	}
	
	
	
	
	// === go_to_wp === //
	
	/**
	 * \brief   action go_to_point setup
	 *
	 * \details this function is devoted to the initialisation of the 
	 * 			service clients 
	 *
	 * \note    For "starting position" we refer to the starting position of 
	 * 			the robot (where it is spawned), coinciding with the temple
	 * 			location. 
	 *
	 *
	 * \return  void
	 */

	void go_to_wp_setup( )
	{
		// (client) /go_to_point : std_srvs/SetBool
		cl_nav = nh.serviceClient<std_srvs::SetBool>("/go_to_point");
	}
	
	/**
	 * \brief	go_to_wp action execution
	 *
	 * \details	once the target waypoint has been assigned, it allows detectibot
	 * 			to move from the current waypoint (less meaningful) toward the
	 * 			next waypoint (more meaningful). Then it retrieves the coordinates
	 * 			associated to the waypoint. It sets the ROS parameters des_pos_x 
	 * 			and des_pos_y. It then call the service  
	 *
	 * \param[in]	msg [rosplan_dispatch_msgs/ActionDispatch] it is the action 
	 * 					dispath message, defined by a name, a specific identifier
	 * 					a plan identifier and other fields 
	 *
	 * \return  Bool True value 
	 */
	bool go_to_wp_action(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		ROS_INFO_STREAM(action_to_string( msg ));
		
		// target waypoint
		std::string wp_to = msg->parameters[0].value;
		
		// call to the navigate_to function 
		navigate_to( wp_to );
		
		return true;
	}
	
	
	
	
	// === shift_gripper === //
	
	/**
	 * \brief   action shift_gripper setup 
	 *
	 * \details this function is devoted to the initialisation of the 
	 * 			service clients 
	 *
	 * \return  void
	 */
	void shift_gripper_setup( )
	{
		// (client) /manipulation : std_srvs/SetBool
		cl_manip = nh.serviceClient<std_srvs::SetBool>( "/manipulation" );
	}
	
	/**
	 * \brief	shift_gripper action execution
	 *
	 * \details	this function allows the detectibot's gripper to move close to the
	 * 			marker so that cluedo_kb node can retrieve the hint via topic. 
	 * 			Then, the boolean message field "data" indicating the fulfillment of the 
	 * 			action (i.e. the activation of the gripper) is set to True and a service
	 * 			call takes place 
	 *
	 * \param[in]	msg [rosplan_dispatch_msgs/ActionDispatch] it is the action 
	 * 					dispath message, defined by a name, a specific identifier
	 * 					a plan identifier and other fields 
	 *
	 * \return  Bool True value 
	 */
	bool shift_gripper_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM(action_to_string( msg ));
		
		
		// prepare the message 
		std_srvs::SetBool cmd;
		cmd.request.data = true;

		// client call to /manipulation service server 
		cl_manip.call( cmd );
		
		return true;
	}
	
	
	
	
	// === gather_hint === //
	
	/**
	 * \brief   action gather_hint setup 
	 *
	 * \details this function is devoted to the initialisation of the 
	 * 			service clients 
	 *
	 * \return  void
	 */
	void gather_hint_setup( )
	{
		// (client) /manipulation : std_srvs/SetBool
		cl_manip = nh.serviceClient<std_srvs::SetBool>( "/manipulation" );
	}
	
	/**
	 * \brief	gather_hint action execution
	 *
	 * \details	this function pull backs the robot's gripper. In the meantime,
	 * 			cluedo_kb receoves the hint. Hence the SetBool message field data
	 * 			is set to "false" and a service call takes place 
	 *
	 *  \note	the service employed for this action is the same employed for
	 * 			the " shift_gripper "  	one. The only differemce is that the data message 
	 * 			field is set to "false"
	 *
	 * \param[in]	msg [rosplan_dispatch_msgs/ActionDispatch] it is the action 
	 * 					dispath message, defined by a name, a specific identifier
	 * 					a plan identifier and other fields 
	 *
	 * \return  Bool True value 
	 */
	bool gather_hint_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		// prepare the message 
		std_srvs::SetBool cmd;
		cmd.request.data = false;
		cl_manip.call( cmd );
		
		return true;
	}
	
	
	
	
	// === reach_temple === //
	
	/**
	 * \brief   action reach_temple setup 
	 *
	 * \details this function is devoted to the initialisation of the 
	 * 			service clients 
	 *
	 * \return  void
	 */
	void reach_temple_setup( )
	{
		// (client) /go_to_point : std_srvs/SetBool
		cl_nav = nh.serviceClient<std_srvs::SetBool>( "/go_to_point" );
	}
	

	
	/**
	 * \brief	reaach_temple action execution
	 *
	 * \details	once the target waypoint has been assigned, it allows detectibot
	 * 			to move from the current waypoint (less meaningful) toward the temple. Then it retrieves the coordinates
	 * 			associated to the waypoint. It sets the ROS parameters des_pos_x 
	 * 			and des_pos_y aaccordingly. Hence, a service call takes place 
	 * 
	 * \note	the temple coordomates correspond to the "center" of the sceme (where)
	 * 			our robot get spawned, initially 
	 *
	 * \param[in]	msg [rosplan_dispatch_msgs/ActionDispatch] it is the action 
	 * 					dispath message, defined by a name, a specific identifier
	 * 					a plan identifier and other fields 
	 *
	 * \return  Bool True value 
	 */
	bool reach_temple_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		// muovi il robot dalla posizione attuale al tempio
		//    imposta des_pos_... a (0, 0)
		//    chiamata a servizio: muovi il robot
		
		navigate_to( "center" );
		
		return true;
	}
	
	
	
	
	// === check_consistent_hypo === //
	
	/**
	 * \brief   action check_consistency setup 
	 *
	 * \details this function is devoted to the initialisation of the 
	 * 			service clients 
	 *
	 * \return  void
	 */
	void check_consistent_hypo_setup( )
	{
		// ... 
	}
	
	/** check_consistent_hypo execution **/
	bool check_consistent_hypo_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		return true;
	}
	
	
	
	
	// === query_hypo === //
	
	/**
	 * \brief   action query_hypo setup 
	 *
	 * \details this function is devoted to the initialisation of the 
	 * 			service clients 
	 *
	 * \return  void
	 */
	void query_hypo_setup( )
	{
		// (client) /get_id : erl_assignment_2_msgs/GetId
		cl_get_id = nh.serviceClient<erl_assignment_2_msgs::GetId>( "/get_id" );
		
		// (client) /oracle_solution : erl2/Oracle
		cl_solution = nh.serviceClient<erl2::Oracle>( "/oracle_solution" );
		
		// (client) /mark_wrong_id : erl_assignment_2_msgs/MarkWrongId
		cl_mark = nh.serviceClient<erl_assignment_2_msgs::MarkWrongId>( "/mark_wrong_id" );
	}
	
	/**
	 * \brief	query_hypo action execution
	 *
	 * \details	this function allows Detectibot to ask for some consistent hypothesis.
	 * 			If they do not exist yet, it returns "false" otherwise a client call to the
	 * 			get_id server takes place. Hence, a check to understand whether or not the 
	 * 			ID is compatible with the one provided by the oracle, is performed. 
	 * 			-	If they are not compatible, a service call to report the 
	 * 				wrongness of the ID is made and a replan occurs. 
	 * 			-	If they results compatible instead, signal that True ID has been found
	 * 				and the game ends
	 * 
	 *
	 *  \note	the service employed for this action is the same employed for
	 * 			the " shift_gripper "  	one. The only differemce is that the data message 
	 * 			field is set to "false"
	 *
	 * \param[in]	msg [rosplan_dispatch_msgs/ActionDispatch] it is the action 
	 * 					dispath message, defined by a name, a specific identifier
	 * 					a plan identifier and other fields 
	 *
	 * \return  Bool True value 
	 */
	bool query_hypo_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		
		// get one consistent hint (if any)
		erl_assignment_2_msgs::GetId id_srv;
		cl_get_id.call( id_srv );
		
		if( !id_srv.response.consistent_found )
		{
			ROS_WARN_STREAM( "(no remaining ids) case UNSOLVABLE." );
			return false;
		}
		
		int c_id = 0;
		if( id_srv.response.consistent_id < 0 )
		{
			ROS_WARN_STREAM( "(no complete hypotheses to propose) NEED FOR REPLAN." );
			
			
			return false;
		}
		else
			c_id = id_srv.response.consistent_id;
		
		// get the solution from the Oracle
		erl2::Oracle solution;
		cl_solution.call( solution );
		
		if( c_id != solution.response.ID  )
		{
			ROS_WARN_STREAM( "(wrong ID) NEED FOR REPLAN." );
			
			
			return false;
		}
		
		ROS_INFO_STREAM( "TRUE ID FOUND! ID=" << c_id );
		
		return true;
	}
	
	
	
	
	// === general purpose privates === //
	
	/** action data to string */
	std::string action_to_string( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		std::string ss = "action name: " + msg->name + "\n";
		ss += "action parameters:";
		
		for( auto it = msg->parameters.begin( ) ; it != msg->parameters.end( ) ; ++it )
			ss += "\n-\t?" + it->key + " : " + it->value;
		
		// ss += "\n";
		return ss;
	}
	
	// ===  	navigation command 	 === //
	/**
	 * \brief   pilot the robot toward a certain position
	 *
	 * \details it retrieves the waypoint coordinates and it sets the
	 * 			target ones within the rosparameter server. 
	 *
	 * \note    (0,0) are the coordinates for the temple's location  
	 *
	 * \param[in]     wp    String value that holds the target point 
	 * 
	 * 
	 *
	 * \return  void
	 */
	void navigate_to( std::string wp )
	{
		// coordinates
		geometry_msgs::Point target = wps[wp];
		
		// set the target into the param server
		ros::param::set( "des_pos_x", target.x );
		ros::param::set( "des_pos_y", target.y );
		
		ROS_INFO_STREAM( "navigation to (" << target.x << ", " << target.y << ")");
		
		// service call
		std_srvs::SetBool cmd;
		cl_nav.call( cmd );
	}
};
}

/** node main function */
int main(int argc, char **argv) 
{
	ros::init(argc, argv, "action_interface", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	
	KCL_rosplan::action_interface_class ac;
	ac.runActionInterface();
	
	return 0;
}
