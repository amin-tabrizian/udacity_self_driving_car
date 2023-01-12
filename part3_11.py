import heapq

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]
print(goal)

delta = [[-1, 0], [0, -1], [1, 0], [0, 1]]
cost = 1

def is_valid(row, column, grid):
    if row < 0 or column < 0 or row >= len(grid) or column >= len(grid[0]):
        return False
    return True

def is_repeated(node, visited_list):
    for search in visited_list:
        if node[1:3] == search[1:3]:
            return True
    return False

class Search:
    def __init__(self, grid, init, goal, delta, cost):
        self.grid = grid
        self.init = init
        self.goal = goal
        self.delta = delta
        self.cost = cost
        self.open_list = []
        heapq.heappush(self.open_list, (0, [0, 0, 0]))
        self.visited_list = []

    def expand(self, node):
        for action in delta:
            i = action[0] + node[1]
            j = action[1] + node[2]
            if is_valid(i, j, self.grid) and not is_repeated([0, i, j], self.visited_list) and not self.grid[i][j]:
                new = [node[0] + 1, i, j]
                if not is_repeated(new, self.visited_list):
                    heapq.heappush(self.open_list, (new[0], new))

    def compute(self):
        while self.open_list:
            node = heapq.heappop(self.open_list)[1]
            if node[1:3] == self.goal:
                print("final", node)
                return
            self.visited_list.append(node)
            self.expand(node)
        print("Fail")
        
a = Search(grid, init, goal, delta, cost)
a.compute()
