import numpy as np
from planner import Planner

class Robot(object):
    def __init__(self, maze_dim, params=[]):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.planner = Planner(maze_dim, params)
        self.maze_dim = maze_dim

        self.race = False #False: trial 1,  True: trial 2
        self.t = 0 #timestep

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        if self.race:
            rotation, movement = self.planner.race(self.t)
        else:
            rotation, movement = self.planner.get_next_move(self.t, sensors)
        self.t += 1

        if self.planner.found_goal and self.planner.found_shortest_path:
            self.race = True
            self.t = 0
            self.planner.visited_goal = False
            self.planner.found_goal = False

        return rotation, movement