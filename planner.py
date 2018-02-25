import numpy as np
import random
from mapper import Mapper
from time import sleep

cardinal_points = ['w', 'n', 'e', 's']
origin_idx, origin_loc = 0, [0, 0]

verbose = True
sleep_on_display = 0.05


class Planner(object):
    def __init__(self, dim, params=[]):
        self.dim = dim
        self.maps = Mapper(dim)
        self.loc = origin_loc
        self.heading = 'n'

        self.found_goal = False
        self.found_goal_idx = 0
        self.found_shortest_path = False

        self.min_percentage = params[0] if params else 40

        self.goal_indices = self.setup_goal_indices(self.dim)
        self.optimal_steps = []

    def setup_goal_indices(self, dim):
        # find indices of four goal locations in the centre of the grid
        goals = [(dim/2, dim/2), (dim/2, dim/2-1), (dim/2-1, dim/2), (dim/2-1, dim/2-1)]
        return [self.loc_to_idx(*l) for l in goals]

    def get_next_move(self, t, sensors):
        # map sensor info to west, north, east, south
        aligned_sensors = self.align_sensors_to_cardinal(sensors)

        # update the knowledge of the map from sensor info
        self.maps.update(self.loc, aligned_sensors)
        p = self.maps.percentage_of_map_visited()

        if verbose:
            print '\n## Step', t, '##', int(p), '%'
            sleep(sleep_on_display)
            self.maps.pretty_print_map(self.loc, self.heading)

        if p >= self.min_percentage and self.found_goal:
            print 'Finished at step', t,
            self.reset()
            rotation, movement = ('Reset', 'Reset')  # start trial 2
        else:
            rotation, movement = self.explore()

        return rotation, movement
    
    def explore(self):
        # get all possible indices that could be visited next
        this_idx = self.loc_to_idx(*self.loc)
        valid_indices = self.get_valid_adj_indices_from_idx(this_idx)

        # rotate 90 degrees if stuck in dead end
        if not len(valid_indices):
            self.heading = cardinal_points[(cardinal_points.index(self.heading) + 1) % 4]
            if verbose:
                print 'idx', this_idx, '--> idx', this_idx, 'rot', 90, 'mov', 0
            return 90, 0

        # filter all indices that weren't visited before
        unvisited_indices = filter(lambda x: x not in self.maps.visited, valid_indices)

        # choose the index with the maximum reward
        if len(unvisited_indices):
            rewards = map(self.maps.get_reward_from_idx, unvisited_indices)
            decision_idx = unvisited_indices[np.argmax(rewards)]
        else: 
            decision_idx = random.choice(valid_indices)

        rotation, movement = self.calculate_move_from_this_to_next_idx(this_idx, decision_idx)
        if verbose:
            print 'idx', this_idx, '--> idx', decision_idx, 'rot', rotation, 'mov', movement

        self.loc = list(self.idx_to_loc(decision_idx))

        if not self.found_goal:
            self.check_goal_found()
        
        return rotation, movement

    def align_sensors_to_cardinal(self, sensors):
        # work out how many steps the robot can take in each direction w, n, e, s
        if self.heading == 'w':
            aligned_sensors = list(sensors[1:] + [0] + [sensors[0]])
        elif self.heading == 'n':
            aligned_sensors = list(sensors + [0])
        elif self.heading == 'e':
            aligned_sensors = list([0] + sensors)
        else:
            aligned_sensors = list([sensors[-1]] + [0] + sensors[:-1])

        return dict(zip(cardinal_points, aligned_sensors))

    def calculate_move_from_this_to_next_idx(self, this_idx, next_idx):
        # work out rotation and movement from one index to the next
        adj_indices = self.get_adj_indices_from_idx(this_idx)
        next_heading, movement = self.heading, 0

        # work out movement
        for cardinal, indices in adj_indices.items():
            if next_idx in indices.keys():
                movement = indices[next_idx]
                next_heading = cardinal

        # work out rotation
        angles = {'w': [-90, 270], 'n': [0], 'e': [90, -270], 's': [180, -180]}
        rotation = -angles[self.heading][0] + angles[next_heading][0]
        rotation = 90 if rotation == -270 else rotation
        rotation = -90 if rotation == 270 else rotation

        self.heading = next_heading
        
        return rotation, movement

    def get_valid_adj_indices_from_idx(self, this_idx):
        # get all adjacent indices
        possible_idx = np.nonzero(self.maps.A1[this_idx, :])[0]
        adj_indices = self.get_adj_indices_from_idx(this_idx)

        # exclude indices that would require 180 degree turn
        ignore_opposed = {'n': 's', 'e': 'w', 's': 'n', 'w': 'e'}
        to_ignore = adj_indices[ignore_opposed[self.heading]].keys()

        return [idx for idx in possible_idx if idx not in to_ignore]

    def get_adj_indices_from_idx(self, this_idx):
        # get all adjacent indices bounded by outer maze walls, excluding itself, ignoring walls of grid
        north = range(this_idx, self.dim ** 2, self.dim)[1:4]
        east = range(this_idx + 1, this_idx + self.dim - this_idx % self.dim)[:3]
        south = range(this_idx, -1, - self.dim)[1:4]
        west = range(this_idx - 1, this_idx - this_idx % self.dim - 1, -1)[:4]

        adj_indices = dict()
        for name, indices in zip(cardinal_points, [west, north, east, south]):
            adj_indices[name] = dict(zip(indices, range(1, len(indices) + 1)))

        return adj_indices

    def check_goal_found(self):
        for goal in self.goal_indices:
            if sum(self.maps.A1[goal]) != 0:
                self.found_goal = True
                self.found_goal_idx = goal

    def calculate_shortest_path_between_indices(self, idx1, idx2): 
        # Floyd-Warshall algorithm
        path = []

        def follow_path(p, i, j):
            if i == j:
                path.append(i)
            else:
                follow_path(p, int(i), int(p[i, j]))
                path.append(j)

        follow_path(self.maps.tree, idx1, idx2)
        return path

    def reset(self):
        self.found_shortest_path = True

        self.loc = origin_loc
        self.heading = 'n'

        self.maps.build_tree_rep()
        shortest_path_indices = self.calculate_shortest_path_between_indices(origin_idx, self.found_goal_idx)
        self.optimal_steps = self.calculate_optimal_steps(shortest_path_indices)
        
        print 'found with path length ', len(self.optimal_steps),
        
        #print 'indices', shortest_path_indices, '\nmoves', self.optimal_steps

    def calculate_optimal_steps(self, shortest_path_indices):
        optimal_steps = []
        for this_idx, next_idx in zip(shortest_path_indices, shortest_path_indices[1:]):
            move = self.calculate_move_from_this_to_next_idx(this_idx, next_idx)
            optimal_steps.append(move)
        return optimal_steps

    def race(self, t):
        return self.optimal_steps[t]

    def loc_to_idx(self, x, y):
        return x + y*self.dim

    def idx_to_loc(self, idx):
        return idx % self.dim, int(idx/self.dim)
