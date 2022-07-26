
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
		wps["wp1"].x = 3.0;
		wps["wp1"].y = 0.0;
		
		wps["wp2"] = geometry_msgs::Point( );
		wps["wp2"].x = 0.0;
		wps["wp2"].y = 3.0;
		
		wps["wp3"] = geometry_msgs::Point( );
		wps["wp3"].x = -3.0;
		wps["wp3"].y = 0.0;
		
		wps["wp4"] = geometry_msgs::Point( );
		wps["wp4"].x = 0.0;
		wps["wp4"].y = -3.0;
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
	
	/** setup action leave_temple */
	void leave_temple_setup( )
	{
		// (client) /go_to_point : std_srvs/SetBool
		cl_nav = nh.serviceClient<std_srvs::SetBool>( "/go_to_point" );
	}
	
	/** leave temple execution */
	bool leave_temple_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
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
	
	/** setup action go_to_wp */
	void go_to_wp_setup( )
	{
		// (client) /go_to_point : std_srvs/SetBool
		cl_nav = nh.serviceClient<std_srvs::SetBool>( "/go_to_point" );
	}
	
	/** go_to_wp execution */
	bool go_to_wp_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		// target waypoint
		std::string wp_to = msg->parameters[0].value;
		
		// dal waypoint attuale (poco interessante) dirigiti al prossimo wp (molto interessante)
		//    trova le coordinate associate al wp
		//    setta i parametri des_pos_x e des_pos_y
		//    chiamata a servizio
		
		navigate_to( wp_to );
		
		return true;
	}
	
	
	
	
	// === shift_gripper === //
	
	/** setup action shift_gripper */
	void shift_gripper_setup( )
	{
		// (client) /manipulation : std_srvs/SetBool
		cl_manip = nh.serviceClient<std_srvs::SetBool>( "/manipulation" );
	}
	
	/** shift_gripper execution */
	bool shift_gripper_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		// avvicina il gripper al marker in modo che cluedo_kb possa ricevere l'hint via topic
		//    prepara il messaggio SetBool con data=true
		//    chiamata a servizio
		
		std_srvs::SetBool cmd;
		cmd.request.data = true;
		cl_manip.call( cmd );
		
		return true;
	}
	
	
	
	
	// === gather_hint === //
	
	/** setup action gather_hint */
	void gather_hint_setup( )
	{
		// (client) /manipulation : std_srvs/SetBool
		cl_manip = nh.serviceClient<std_srvs::SetBool>( "/manipulation" );
	}
	
	/** gather_hint execution */
	bool gather_hint_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		// tira indietro il braccio (cluedo_kb ha ricevuto l'hint)
		//    prepara il messaggio con setbool data=false
		//    chiamata a servizio
		
		std_srvs::SetBool cmd;
		cmd.request.data = false;
		cl_manip.call( cmd );
		
		return true;
	}
	
	
	
	
	// === reach_temple === //
	
	/** setup action reach_temple */
	void reach_temple_setup( )
	{
		// (client) /go_to_point : std_srvs/SetBool
		cl_nav = nh.serviceClient<std_srvs::SetBool>( "/go_to_point" );
	}
	
	/** reach_temple execution **/
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
	
	/** setup action check_consistent_hypo */
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
	
	/** setup action check_consistent_hypo */
	void query_hypo_setup( )
	{
		// (client) /get_id : erl_assignment_2_msgs/GetId
		cl_get_id = nh.serviceClient<erl_assignment_2_msgs::GetId>( "/get_id" );
		
		// (client) /oracle_solution : erl2/Oracle
		cl_solution = nh.serviceClient<erl2::Oracle>( "/oracle_solution" );
		
		// (client) /mark_wrong_id : erl_assignment_2_msgs/MarkWrongId
		cl_mark = nh.serviceClient<erl_assignment_2_msgs::MarkWrongId>( "/mark_wrong_id" );
	}
	
	/** check_consistent_hypo execution **/
	bool query_hypo_action( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
	{
		ROS_INFO_STREAM( action_to_string( msg ) );
		
		// chiamata a servizio : chiedi se ci sono ipotesi consistenti
		//    se non ce ne sono, return false
		// chiamata a servizio
		// verifica se l'id scelto è compatibile con quello ritornato dall'oracolo
		//    (se no), chiamata a servizio : segnala l'ID come errato
		//    e replan
		//    return false
		// altrimenti c'hai azzeccato!
		//    return true
		
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
			
			/// @todo replan strategy
			
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
			
			/// @todo replan strategy
			
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
	
	/** navigation command */
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
