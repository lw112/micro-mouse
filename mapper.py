import numpy as np
from itertools import permutations

#
#  key for maze decryption
#
#  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
#  _       _       _       _       _       _       _       _
# |_| |_| |_  |_  | | | | |   |    _|  _|  _   _    |   |     
#
#
#  indices system
#   __ __ __ __ __ __     __ __
#  |60|61|62|63|64|65 ... 70|71|
#  |48|49|50|51|52|53 ... 58|59|
#  |36|37|38|39|40|41 ... 46|47|
#  |24|25|26|27|28|29 ... 34|35|
#  |12|13|14|15|16|17 ... 22|23|
#  |0 |1 |2 |3 |4 |5  ... 10|11|
#   -- -- -- -- -- --     -- --

closed = {'w': [0, 1, 2, 3, 4, 5, 6, 7],
          's': [0, 1, 2, 3, 8, 9, 10, 11], 
          'e': [0, 1, 4, 5, 8, 9, 12, 13], 
          'n': [0, 2, 4, 6, 8, 10, 12, 14]}


class Mapper(object):
    def __init__(self, dim):
        self.dim = dim
        self.walls = np.zeros(shape=(self.dim, self.dim), dtype=int)  # walls of maze
        self.visited = set()  # set of visited indices

        self.A1 = np.zeros(shape=(self.dim ** 2, self.dim ** 2), dtype=int)  # adjacency matrix
        self.tree = np.zeros(shape=(self.dim ** 2, self.dim ** 2), dtype=int)  # adjacency tree

        self.R = self.setup_reward_grid()  # reward grid

    def setup_reward_grid(self):
        r = self.dim / 2
        grid = np.full((self.dim, self.dim), -r)
        for c, x in enumerate(range(r - 1, 0, -1)):
            sub = np.full((x * 2, x * 2), -x)
            idx = c + 1
            grid[idx:-idx, idx: -idx] = sub
        
        return grid

    def update(self, loc, sensors):
        # given aligned sensors
        this_idx = loc[0] + loc[1]*self.dim
        self.visited.add(this_idx)
        
        adj_idx = []
        limit = 3  # maximum number of steps the robot can take

        # facing north
        for i in range(sensors['n']):
            x1, y1, x2, y2 = loc[0], loc[1] + i, loc[0], loc[1] + i + 1

            if self.walls[x1, y1] in closed['n']:
                self.walls[x1, y1] += 1  # opens top of this field
                self.walls[x1, y2] += 4  # opens bottom of next field
                
                mult = np.clip(sensors['n'] - i, 0, limit)
                idx = [x1 + (y1 + y)*self.dim for y in range(mult + 1)]
                adj_idx += list(permutations(idx, 2))

        # facing east
        for i in range(sensors['e']):
            x1, y1, x2, y2 = loc[0] + i, loc[1], loc[0] + i + 1, loc[1]
            if self.walls[x1, y1] in closed['e']:
                self.walls[x1, y1] += 2  # opens right of this field
                self.walls[x2, y2] += 8  # opens left of next field

                mult = np.clip(sensors['e'] - i, 0, limit)
                idx = [x1 + x + y1*self.dim for x in range(mult + 1)]
                adj_idx += list(permutations(idx, 2))

        # facing south
        for i in range(sensors['s']):
            x1, y1, x2, y2 = loc[0], loc[1] - i, loc[0], loc[1] - i - 1
            if self.walls[x1, y1] in closed['s']:
                self.walls[x1, y1] += 4  # opens bottom of this field
                self.walls[x2, y2] += 1  # opens top of next field
                
                mult = np.clip(sensors['s'] - i, 0, limit)
                idx = [x1 + (y1 - y)*self.dim for y in range(mult + 1)]
                adj_idx += list(permutations(idx, 2))

        # facing west
        for i in range(sensors['w']):
            x1, y1, x2, y2 = loc[0] - i, loc[1], loc[0] - i - 1, loc[1]
            if self.walls[x1, y1] in closed['w']:
                self.walls[x1, y1] += 8  # opens left of this field
                self.walls[x2, y2] += 2  # opens right of next field
                
                mult = np.clip(sensors['w'] - i, 0, limit)
                idx = [x1 - x + y1*self.dim for x in range(mult + 1)]
                adj_idx += list(permutations(idx, 2))

        for (x, y) in adj_idx:
            self.A1[x, y] = 1

    def percentage_of_map_visited(self):
        return 100/(self.dim**2.0)*len(self.visited)

    def build_tree_rep(self):
        adj, dim2 = self.A1, self.dim ** 2
        tree = np.zeros(adj.shape)
        for i in range(dim2):
            for j in range(dim2):
                tree[i, j] = i
                if i != j and adj[i, j] == 0:
                    tree[i, j] = -10000 
                    adj[i, j] = 10000

        for k in range(dim2):
            for i in range(dim2):
                for j in range(dim2):
                    if adj[i, j] > adj[i, k] + adj[k, j]:
                        adj[i, j] = adj[i, k] + adj[k, j]
                        tree[i, j] = tree[k, j]
        self.tree = tree

    def get_reward_from_idx(self, idx):
        x, y = self.idx_to_loc(idx)
        return self.R[x, y]

    def idx_to_loc(self, idx):
        return idx % self.dim, int(idx/self.dim)

    def pretty_print_map(self, loc, heading):
        dim = self.dim
        v, h, o, n = '|', '_', ' ', '\n'

        # note: cannot print the bottom wall for current field since underlined 'v' not possible
        robot_heading = {'w': '<', 'n': '^', 'e': '>', 's': 'v'}

        x_dim, y_dim = dim * 2 + 3, dim + 1

        top_row = [o] + list([h, o]) * dim + [o] + [n]
        grid = [top_row]

        for y in range(0, dim):
            wall = self.walls[:, -y-1]
            row = [' ']*x_dim
            for x, bit in enumerate(wall):
                row[x*2] = v if bit in closed['w'] else o
                row[x*2 + 1] = h if bit in closed['s'] else o  
            row[-3], row[-1] = v, n
            grid.append(row)

        grid[y_dim - loc[1] - 1][loc[0]*2 + 1] = robot_heading[heading]

        g_dim = len(grid)
        print_grid = '\n'
        for idx, line in enumerate(grid):
            print_grid += str(g_dim - idx - 1).zfill(2) + ' ' + ''.join(line)

        coordinates = [str(x % 10) + ' ' for x in range(g_dim)]
        print_grid += '    ' + ''.join(coordinates) + '\n'
        print print_grid
