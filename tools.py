def pretty_print_map(dim, loc, heading):
    
    v, h, o, n = '|', '_', ' ', '\n'

    # note: cannot print the bottom wall for current field since underlined 'v' not possible
    robot_heading = {'w': '<', 'n': '^', 'e': '>', 's': 'v'}

    x_dim, y_dim = dim * 2 + 3, dim + 1

    top_row = [o] + list([h, o]) * dim + [o] + [n]
    grid = [top_row]

    for y in range(0, dim):
        wall = walls[:, -y-1]
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
