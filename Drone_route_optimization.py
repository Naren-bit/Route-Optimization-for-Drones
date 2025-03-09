import heapq

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Rectangle:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def contains(self, point):
        return (self.x <= point.x <= self.x + self.w) and (self.y <= point.y <= self.y + self.h)

    def intersects(self, range_rect):
        """Check if this rectangle intersects with another rectangle (range_rect)."""
        return not (range_rect.x > self.x + self.w or
                    range_rect.x + range_rect.w < self.x or
                    range_rect.y > self.y + self.h or
                    range_rect.y + range_rect.h < self.y)

class QuadTree:
    def __init__(self, boundary, capacity):
        self.boundary = boundary  
        self.capacity = capacity  
        self.points = []
        self.divided = False

    def subdivide(self):
        x, y, w, h = self.boundary.x, self.boundary.y, self.boundary.w / 2, self.boundary.h / 2
        self.northeast = QuadTree(Rectangle(x + w, y, w, h), self.capacity)
        self.northwest = QuadTree(Rectangle(x, y, w, h), self.capacity)
        self.southeast = QuadTree(Rectangle(x + w, y + h, w, h), self.capacity)
        self.southwest = QuadTree(Rectangle(x, y + h, w, h), self.capacity)
        self.divided = True

    def insert(self, point):
        if not self.boundary.contains(point):
            return False
        if len(self.points) < self.capacity:
            self.points.append(point)
            return True
        else:
            if not self.divided:
                self.subdivide()
            return (self.northeast.insert(point) or
                    self.northwest.insert(point) or
                    self.southeast.insert(point) or
                    self.southwest.insert(point))

    def query(self, range_rect, found):
        if not self.boundary.intersects(range_rect):
            return
        for p in self.points:
            if range_rect.contains(p):
                found.append(p)
        if self.divided:
            self.northwest.query(range_rect, found)
            self.northeast.query(range_rect, found)
            self.southwest.query(range_rect, found)
            self.southeast.query(range_rect, found)

class SegmentTree:
    def __init__(self, size):
        self.size = size
        self.tree = [float('inf')] * (2 * size)  

    def update(self, index, value):
        index += self.size
        self.tree[index] = value
        while index > 1:
            index //= 2
            self.tree[index] = min(self.tree[2 * index], self.tree[2 * index + 1])

    def query(self, left, right):
        left += self.size
        right += self.size
        res = float('inf')
        while left < right:
            if left % 2 == 1:
                res = min(res, self.tree[left])
                left += 1
            if right % 2 == 1:
                right -= 1
                res = min(res, self.tree[right])
            left //= 2
            right //= 2
        return res

# Node class for A* pathfinding

class Node:
    def __init__(self, position, g=0, h=0, f=0):
        self.position = position
        self.g = g  
        self.h = h  
        self.f = f  

    def __lt__(self, other):
        return self.f < other.f

def reconstruct_path(came_from, start, goal):
    path = [goal]
    while goal != start:
        goal = came_from[goal]
        path.append(goal)
    path.reverse()
    return path

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar_with_obstacle_handling(start, goal, quad_tree, segment_tree, graph, heuristic):
    open_list = []
    heapq.heappush(open_list, Node(start, 0, heuristic(start, goal), heuristic(start, goal)))
    came_from = {}
    cost_so_far = {start: 0}

    while open_list:
        current_node = heapq.heappop(open_list)
        current = current_node.position

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        for neighbor, cost in graph.get(current, []):
            
            if segment_tree.query(neighbor[0], neighbor[0] + 1) != float('inf'):
                continue  
            
            found_obstacles = []
            search_area = Rectangle(neighbor[0] - 5, neighbor[1] - 5, 10, 10)
            quad_tree.query(search_area, found_obstacles)

            if not found_obstacles:
                new_cost = cost_so_far[current] + cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_list, Node(neighbor, new_cost, heuristic(neighbor, goal), priority))
                    came_from[neighbor] = current

    return None  


def handle_dynamic_obstacles(segment_tree, quad_tree, new_obstacle_position):
    segment_tree.update(new_obstacle_position[0], new_obstacle_position[1])
    quad_tree.insert(Point(new_obstacle_position[0], new_obstacle_position[1]))

if __name__ == "__main__":
    
    boundary = Rectangle(0, 0, 100, 100)
    quad_tree = QuadTree(boundary, 4)
    segment_tree = SegmentTree(100)

    graph = {
        (0, 0): [((1, 0), 1), ((0, 1), 1)],
        (1, 0): [((0, 0), 1), ((1, 1), 1)],
        (0, 1): [((0, 0), 1), ((1, 1), 1)],
        (1, 1): [((1, 0), 1), ((0, 1), 1), ((2, 2), 1.5)],
        (2, 2): [((1, 1), 1.5)]
    }

    obstacles = [Point(10, 15), Point(50, 50), Point(70, 80)]
    for obstacle in obstacles:
        quad_tree.insert(obstacle)
        segment_tree.update(obstacle.x, obstacle.y)

    handle_dynamic_obstacles(segment_tree, quad_tree, (30, 40))

    start = (0, 0)
    goal = (2, 2)
    path = astar_with_obstacle_handling(start, goal, quad_tree, segment_tree, graph, heuristic)

    print("Optimal path avoiding obstacles:", path)
    