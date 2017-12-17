#!/usr/bin/env python
import rospy
import smach
import smach_ros
from me212_robot.msg import smachAction, smachResult, smachFeedback, smachGoal
import actionlib
from std_msgs.msg import Float64MultiArray
x = 0
y = 0
yaw = 0

# define state
class XXXXXXXXX(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['XXXXXXXXX'])


    def execute(self, userdata):
        rospy.loginfo('Executing state XXXXXXXXX')

        # Write down what you want to do
 
        return 'XXXXXXXXX'   # return your outcome


# define state
class XXXXXXXXX(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['XXXXXXXXX','XXXXXXXXX'])

        # Set_up_ros_action (if you need to use smach_action)
        self.client = actionlib.SimpleActionClient('smach_action', smachAction)
        self.client.wait_for_server()


    def execute(self, userdata):
        rospy.loginfo('Executing state XXXXXXXXX')


        # Write down what you want to do or your policy for outcome
        if XXXXXXXXX:

            return 'XXXXXXXXX'  # return your outcome
        else:
            return 'XXXXXXXXX'  # return your outcome


############### call action for specific motion ###############
    def pub_action_goal(self, distance):      
        # Creates a goal to send to the action server.
        goal = smachGoal(action_id=0, number=distance)
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
################################################################

# define state
class XXXXXXXXX(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['XXXXXXXXX'])
        # Set_up_ros_action
        self.client = actionlib.SimpleActionClient('smach_action', smachAction)

    def execute(self, userdata):
        rospy.loginfo('Executing state XXXXXXXXX')

        # Write down what you want to do or your policy for outcome

        return 'XXXXXXXXX'
        
############### call action for specific motion ###############
    def pub_action_goal(self, distance):
        # Creates a goal to send to the action server.
        goal = smachGoal(action_id=1, number=distance)
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
################################################################
        
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('XXXXXXXXX', Keyboard_move(), 
                               transitions={'XXXXXXXXX':'XXXXXXXXX'})
        smach.StateMachine.add('XXXXXXXXX', GO_Straight(), 
                               transitions={'XXXXXXXXX':'XXXXXXXXX', 'XXXXXXXXX':'outcome4'})
        smach.StateMachine.add('XXXXXXXXX', Go_circle(), 
                               transitions={'XXXXXXXXX':'XXXXXXXXX'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/LAB10_ROS_SMACH')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

def cbPose(msg):
    global x,y,yaw
    x = msg.data[0]
    y = msg.data[1]
    yaw = msg.data[2]
    # print x,y,yaw

if __name__ == '__main__':
    sub_odom = rospy.Subscriber("/odometry", Float64MultiArray, cbPose)
    main()
