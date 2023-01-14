import numpy as np

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]
print(goal)

delta = [[-1, 0],
         [0, -1],
         [1, 0],
         [0, 1]]
delta_name = ['^', '<', 'v', '>']
cost = 1

def is_valid(row, column, grid):
    if row < 0 or column < 0 or row >= len(grid) or column >= len(grid[0]) or grid[row][column]:
        return False
    return True

def is_repeated(node, visited_list):
    for search in visited_list:
        if node[1:3] == search[1:3]:
            return True
    return False

class Search:
    def __init__(self, grid, init, goal, delta, cost, delta_name):
        self.grid = grid
        self.init = init
        self.goal = goal
        self.delta = delta
        self.cost = cost
        self.delta_name = delta_name
        self.open_list = [[0, 0, 0]]
        self.visited_list = []
        self.expansion_grid = -1*np.ones((len(grid), len(grid[0])))
        self.step = 0
    def expand(self, node):
        for action in delta:
            i = action[0] + node[1]
            j = action[1] + node[2]
            if is_valid(i, j, self.grid) and not is_repeated([0, i, j], self.visited_list):
                new = [node[0] + 1, i, j]
                if not is_repeated(new, self.open_list):
                    self.open_list.append(new)
        # print("Open list updated:")
        # print(self.open_list)        
        self.expansion_grid[node[1], node[2]] = self.step
        self.step += 1
        # print(self.step)
    def compute_policy(self):
        policy = np.zeros((len(self.grid), len(self.grid[0])), dtype=int)
        # policy[:] =
        for i in range(len(self.grid)):
            max = 0
            for j in range(len(self.grid[0])):
                for idx, action in enumerate(delta):
                    if is_valid(i, j, self.grid):
                       new_i =  action[0] + i
                       new_j = action[1] + j
                    if is_valid(new_i, new_j, self.grid):
                        if max < self.expansion_grid[new_i, new_j]:
                            policy[i, j] = idx
                            max = self.expansion_grid[new_i, new_j]
        return policy
    def compute(self, node = [0, 0, 0]):
        self.open_list.remove(node)
        self.visited_list.append(node)
        while  node[1:3] != self.goal:
            self.expand(node)
            self.open_list = sorted(self.open_list, key=lambda t: t[0])
            if self.open_list == []:
                print("Fail")
                return
            node = self.open_list[0]
            self.open_list.remove(node)
            self.visited_list.append(node)
        print("final", node)
        self.expansion_grid[node[1], node[2]] = self.step
    def run_policy(self, policy):
        path = np.chararray(policy.shape)
        path[:] = ''
        node = [0, 0]
        n_node = node.copy()
        while node != self.goal:
            print(node)
            path[node[0], node[1]] = delta_name[policy[node[0], node[1]]]
            n_node[0] = node[0] + self.delta[policy[node[0], node[1]]][0]
            n_node[1] = node[1] + self.delta[policy[node[0], node[1]]][1]
            node = n_node.copy()
        path[node[0], node[1]] = '*'
        return path
a = Search(grid, init, goal, delta, cost, delta_name)
# print(a.open_list)
# a.expand([0, 0, 0])
# print(a.open_list)
a.compute()
print(a.expansion_grid)
print(a.compute_policy())
print(a.run_policy(a.compute_policy()).decode())
