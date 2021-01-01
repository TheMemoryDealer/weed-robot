#! /usr/bin/env python
import rospy
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String
from colorama import Fore, Back, Style, init
init(autoreset=True)


class Navigation:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()
        self.goal = GotoNodeGoal()
        self.pub = rospy.Publisher('crop_difficulty', String, queue_size=1)

    def execute(self):
        waypoints = ['Easy_1_start', 'Easy_1_finish', 'Easy_2_start', 'Easy_2_finish',
        'Medium_1_start', 'Medium_1_finish', 'Medium_2_start', 'Medium_2_finish',
        'Hard_1_start', 'Hard_1_finish', 'Hard_2_start', 'Hard_2_finish']
        
        ccounter = 0

        for waypoint in waypoints:
            self.goal.target = waypoint
            self.client.send_goal(self.goal)
            status = self.client.wait_for_result()
            result = self.client.get_result()
            # rospy.loginfo("status for %s is %s", status, self.goal.target)
            # rospy.loginfo("result is %s", result)
            # print(Fore.RED + 'some red text') 
            # rospy.loginfo(Fore.RED + self.goal.target)
            if('2_finish' in self.goal.target):
                print('Plant env cleared! proceeding to next')
                self.pub.publish('NA')
                continue
        
            if ('Easy' in self.goal.target):
                print('Current plant env: '+ Fore.GREEN + 'Easy')
                self.pub.publish('Easy')
            elif ('Medium' in self.goal.target):
                print('Current plant env: '+ Fore.YELLOW + 'Medium')
                self.pub.publish('Medium')
            elif ('Hard' in self.goal.target):
                self.pub.publish('Hard')
                print('Current plant env: '+ Fore.RED + 'Hard')

            




if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    print('-----Navigation started-----')
    Navigation().execute()
    print('-----Navigation exiting-----')
