#!/bin/bash

import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceRequest, DispatchServiceResponse

if __name__ == "__main__":
	rospy.init_node( "main" )
	
	em = Empty( )
	dis = DispatchServiceResponse( )
	
	# planning triggers
	cl_problem_interface = rospy.ServiceProxy( "/rosplan_problem_interface/problem_generation_server", Empty )
	cl_planner_interface = rospy.ServiceProxy( "/rosplan_planner_interface/planning_server", Empty )
	cl_parsing_interface = rospy.ServiceProxy( "/rosplan_parsing_interface/parse_plan", Empty )
	cl_plan_dispatcher = rospy.ServiceProxy( "/rosplan_plan_dispatcher/dispatch_plan", DispatchService )
	
	solved = False
	
	while not solved:
		rospy.loginfo( "replanning..." )
		cl_problem_interface( )
		cl_planner_interface( )
		cl_parsing_interface( )
		
		rospy.loginfo( "dispatching plan..." )
		dis = cl_plan_dispatcher( )
		
		solved = (dis.success and dis.goal_achieved)
		if solved:
			rospy.loginfo( "mystery solved." )
		else:
			rospy.loginfo( "still not solved" )
