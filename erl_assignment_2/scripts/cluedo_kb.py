
'''

'''

import rospy

NODE_NAME = "cluedo_kb"

LOGSQUARE = lambda mstr : f"[{mstr}] "
OUTLABEL = LOGSQUARE( NODE_NAME )
TLOG = lambda m : rospy.loginfo( OUTLABEL + str( m ) )
TWARN = lambda m : rospy.logwarn( OUTLABEL + "WARNING: " + str( m ) )
TERR = lambda m : rospy.logerr( OUTLABEL + "ERROR" + str( m ) )

record_where = 0
record_what = 1
record_who = 2


class classnode_cluedo_kb:
    
    def __init__( self ):
        '''Node Constrctor
        
        TODO:
            implement me!
        '''
        pass
    
    def spin( self ):
        '''The main functionality of the node
        
        TODO:
            implement me!
        '''
        
        '''
        r = rospy.Rate( 5 )
        while not rospy.is_shutdown():
            pass
        '''
        
        rospy.spin( )
        pass
    
    pass # ...



def shut_msg( ):
    TLOG( "stopping ... " )



if __name__ == "__main__":
    rospy.init_node( NODE_NAME )
    rospy.on_shutdown( shut_msg )
    
    TLOG( "starting ... " )
    
    # TODO define here messages and services ...

    TLOG( "ready" )
    
    # TODO here the main functionality ...
    ( classnode_cluedo_kb( ) ).spin( )
