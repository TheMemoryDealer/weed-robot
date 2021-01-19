#! /usr/bin/env python
import rospy
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String
from colorama import Fore, Back, Style, init # this allows for color print to console
init(autoreset=True) # dont save color to memory


class Navigation:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction) # subscribe to movement master
        self.client.wait_for_server() 
        self.goal = GotoNodeGoal()
        self.pub = rospy.Publisher('/thorvald_001/crop_difficulty', String, queue_size=1) # publish which crop going over

    def execute(self):
        
        waypoints = [ # define goals to move to
        'Easy_1_start', 'Easy_1_10%', 'Easy_1_20%', 'Easy_1_30%', 'Easy_1_40%','Easy_1_50%', 'Easy_1_60%', 'Easy_1_70%', 'Easy_1_80%','Easy_1_90%', 'Easy_1_100%', 'Easy_1_finish',
        'Easy_2_start', 'Easy_2_10%', 'Easy_2_20%', 'Easy_2_30%', 'Easy_2_40%','Easy_2_50%', 'Easy_2_60%', 'Easy_2_70%', 'Easy_2_80%','Easy_2_90%','Easy_2_100%', 'Easy_2_finish',
        'Medium_1_start','Medium_1_10%','Medium_1_20%','Medium_1_30%','Medium_1_40%','Medium_1_50%','Medium_1_60%','Medium_1_70%','Medium_1_80%','Medium_1_90%', 'Medium_1_100%', 'Medium_1_finish',
        'Medium_2_start','Medium_2_start','Medium_2_10%','Medium_2_20%','Medium_2_30%','Medium_2_40%','Medium_2_50%','Medium_2_60%','Medium_2_70%','Medium_2_80%','Medium_2_90%', 'Medium_2_100%', 'Medium_2_finish',
        'Hard_1_start', 'Hard_1_10%', 'Hard_1_20%', 'Hard_1_30%', 'Hard_1_40%', 'Hard_1_50%' ,'Hard_1_60%', 'Hard_1_70%', 'Hard_1_80%', 'Hard_1_90%', 'Hard_1_finish',
        'Hard_2_start', 'Hard_2_10%', 'Hard_2_20%', 'Hard_2_30%', 'Hard_2_40%', 'Hard_2_50%', 'Hard_2_60%', 'Hard_2_70%', 'Hard_2_80%', 'Hard_2_90%', 'Hard_2_100%', 'Hard_2_finish']
        
        for waypoint in waypoints: # loop through waypoints
            self.goal.target = waypoint # set target
            self.client.send_goal(self.goal) # send target
            status = self.client.wait_for_result() # wait for result
            result = self.client.get_result() # receive result
            
            ''' 100% milestone is a bit of a hack. It is meant to solve the PCL problem where Thor would 
            continue seeing weeds for about 0.5 meters outside of the crop area due to FPS lag + camera
            rotation/position. To bypass that, we're saying hey Thor, imagine the weeds end 0.5 meters
            before the actual end of the row. It wont affect accuracy too much as your rotated camera
            can still spot those weeds in the last few frames. Makes pointcloud map look a bit nice'''
            
            if('finish' in self.goal.target or '100' in self.goal.target): # no crop on end of rows
                self.pub.publish('N/A')
        
            if ('Easy' in self.goal.target and '%' not in self.goal.target and 'finish' not in self.goal.target): # check for crop type and not milestone
                print('Current plant env: '+ Fore.GREEN + 'Easy')
                self.pub.publish('Easy') # publish for vision
                
            elif ('Medium' in self.goal.target and '%' not in self.goal.target and 'finish' not in self.goal.target): # check for crop type and not milestone
                print('Current plant env: '+ Fore.YELLOW + 'Medium')
                self.pub.publish('Medium') # publish for vision
                
            elif ('Hard' in self.goal.target and '%' not in self.goal.target and 'finish' not in self.goal.target): # check for crop type and not milestone
                print('Current plant env: '+ Fore.RED + 'Hard')
                self.pub.publish('Hard') # publish for vision

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client') # initialize this class as ROS node
    print('-----Navigation started-----')
    Navigation().execute() # run main func
    print('-----Navigation exiting-----')
