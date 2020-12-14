#! /usr/bin/env python
import rospy
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from colorama import Fore, Back, Style, init
init(autoreset=True)


class Navigation:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()
        self.goal = GotoNodeGoal()

    def execute(self):
        waypoints = ['Easy_1_start', 'Easy_1_finish', 'Easy_2_start', 'Easy_2_finish',
        'Medium_1_start', 'Medium_1_finish', 'Medium_2_start', 'Medium_2_finish',
        'Hard_1_start', 'Hard_1_finish', 'Hard_2_start', 'Hard_2_finish']

        for waypoint in waypoints:
            self.goal.target = waypoint
            self.client.send_goal(self.goal)
            self.lane_bound_monitor()
            self.lane_dificulty_monitor()

    def lane_dificulty_monitor(self):
        if ('Easy' in self.goal.target):
            print('Current plant env: '+ Fore.GREEN + 'Easy')
        elif ('Medium' in self.goal.target):
            print('Current plant env: '+ Fore.YELLOW + 'Medium')
        elif ('Hard' in self.goal.target):
            print('Current plant env: '+ Fore.RED + 'Hard')

    def lane_bound_monitor(self):
        if ('2_finish' in self.goal.target):
            print('Env cleared! proceeding to next')
        elif ('1_start' in self.goal.target):
            print('Begin clearing for new env')




if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    Navigation().execute()
