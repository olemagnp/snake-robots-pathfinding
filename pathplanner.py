import numpy as np
import environment as e
import matplotlib.pyplot as plt
import math
from collections import deque
from heapq import *

def create_desired_path(env, num_wp):
    points = []
    start_p = Point(*env.start_pos)
    end_p = Point((np.random.rand()*0.1 + 0.9 )*env.width, \
                     (np.random.rand()*0.7 + 0.15)*env.height)
    points.append(start_p)
    dist_x = end_p.x - start_p.x
    dist_y = end_p.y - start_p.y
    
    for i in range(1, num_wp-1) :
        p = Point(start_p.x,start_p.y)
        p.x += dist_x*i/num_wp
        p.y += dist_y*i/num_wp + np.random.rand()*dist_x/(num_wp)

        p.y = max(p.y, 0.1*env.height)
        p.y = min(p.y, 0.9*env.height)
        
        points.append(p)
    points.append(end_p)
    return points

def plot_desired_path(points, path=None, env=None, fig=None):
    # Plot the environment and use its figure to plot the desired path
    fig = plt.figure() if fig is None else fig
    if env:
        fig = e.plot_environment(env, path=path, fig=fig)
        ax = fig.gca()
        ax.set_xlim([0, env.width])
        ax.set_ylim([0, env.height])
    else:
        ax = fig.add_subplot(111)
    x = []
    y = []
    for p in points:
        x.append(p.x)
        y.append(p.y)
    ax.plot(x,y,'go-')
    plt.show()


class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.distance = None

    def distance_to(self, p):
        return np.sqrt((self.x- p.x) ** 2 + (self.y-p.y) ** 2)
    
    def __repr__(self):
        return "Point(%.2f, %.2f)" % (self.x, self.y)
    
    def __str__(self):
        return self.__repr__()
    
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

class Line:
    def __init__(self, length, angle):
        self.length = length
        self.angle = angle

    def angle_to_line(self, line):
        return line.angle - self.angle

    def __repr__(self):
        return "Line[length=%.2f, angle=%.2f]" % (self.length, self.angle)

    def __str__(self):
        return self.__repr__()


class Triplet:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.side = 0
        self.cost = 0 # Actual cost to move to this obstacle 
        self.h = 0 # Heuristic cost left to the goal
        self.parent = None

    def push_front(self,ob):
        self.obstacles[0] = self.obstacles[1]
        self.obstacles[1] = self.obstacles[2]
        self.obstacles[2] = ob
    
    def get_next_triplet(self, ob):
        trip = Triplet(self.obstacles[1:] + [ob])
        trip.side = (self.side + 1) % 2
        return trip
    
    def __eq__(self, o):
        return self.side == o.side and np.all(self.obstacles == o.obstacles)
    
    def __lt__(self, o):
        return self.cost + self.h < o.cost + o.h

    def __le__(self, o):
        return self.cost + self.h <= o.cost + o.h
        self.side = 0                   ### side = 0 - snake is right of object 3.
                                        ### side = 1 - snake is left of object 3.
        self.distance_to_next=None
    
    def quadruplet_distance_stupid(self, neighbours, snake_length):
        point2 = Point(self.obstacles[0].x, self.obstacles[0].y)
        point1 = Point(self.obstacles[1].x, self.obstacles[1].y)
        point0 = Point(self.obstacles[2].x, self.obstacles[2].y)

        snake_left = snake_length - point0.distance_to(point1) - point1.distance_to(point2)
        reachable_pts = {}
        for obstacle, distance in neighbours.items():
            if snake_left >= distance:
                reachable_pts[obstacle] = distance
        return reachable_pts

    def quadruplet_distance(self, neighbours, snake_length):
        tangent_lines = []

        for i in range(2):
            center_distance = np.sqrt((self.obstacles[i+1].x - self.obstacles[i].x) ** 2 + (self.obstacles[i+1].y - self.obstacles[i].y) ** 2)
            center_theta = np.arctan2((self.obstacles[i+1].y - self.obstacles[i].y), (self.obstacles[i+1].x - self.obstacles[i].x))

            summed_radius = self.obstacles[i].radius + self.obstacles[i+1].radius

            tangent_length = np.sqrt(center_distance ** 2 - summed_radius ** 2)
            theta_difference = np.arcsin(summed_radius / center_distance)

            tangent_lines.append([Line(tangent_length, center_theta - theta_difference), Line(tangent_length, center_theta + theta_difference)])

        ob_dict = {}

        for ob in neighbours:
            if ob in self.obstacles:
                continue
            center_distance = np.sqrt((ob.x - self.obstacles[2].x) ** 2 + (ob.y - self.obstacles[2].y) ** 2)
            center_theta = np.arctan2((ob.y - self.obstacles[2].y), (ob.x - self.obstacles[2].x))

            summed_radius_neigh = (self.obstacles[2].radius + ob.radius)

            tangent_length = np.sqrt(center_distance ** 2 - summed_radius_neigh ** 2)
            theta_difference = np.arcsin(summed_radius_neigh / center_distance)
            tangent_lines_neigh = ([Line(tangent_length, center_theta - theta_difference), Line(tangent_length, center_theta + theta_difference)])

            s = self.side
            lines = [tangent_lines[0][(s+1)%2], tangent_lines[1][s], tangent_lines_neigh[(s+1)%2]]
            radii = [self.obstacles[0].radius, self.obstacles[1].radius, self.obstacles[2].radius, ob.radius]
            path_length = length_of_three_lines(lines, radii, s)

            if path_length < snake_length:
                ob_dict[ob] = path_length
        return ob_dict  # dictionary of {Obstacle: required snake length}

def find_start_end_points(goal_ob, tangent, side):
    radial_theta = line.angle + math.pi/2 if side == 1 else line.angle - math.pi/2
    end_x = goal_ob.x + goal_ob.radius * np.cos(radial_theta)
    end_y = goal_ob.y + goal_ob.radius * np.sin(radial_theta)

    start_x = end_x - line.length * np.cos(line.angle)
    start_y = end_y - line.length * np.sin(line.angle)
    return Point(start_x, start_y), Point(end_x, end_y)

def length_of_three_lines(lines, radii, s):  # s is side
    path_length = 0
    for line in lines:
        path_length += line.length

    pl = [path_length, 0, 0]

    theta = [lines[0].angle_to_line(lines[1]), lines[1].angle_to_line(lines[2])]

    for i in range(2):

        if (theta[i] <= 0) == (((s+i) % 2) == 0):  # OUTSIDE
            path_length += radii[i+1]*abs(theta[i])
        else:  # INSIDE
            theta[i] = abs(theta[i])
            # This code finds a point on the middle of the arch between the points where the two tangent lines intersects with the circle.
            # Then it finds the distances from the endpoints to the point on the arch, and adds these to the path_length. Then it subtracts the length of the tangent lines from
            # path_length, as these are added earlier in the code. (See picture on google drive)
            a = np.sqrt((radii[i+1] * (1 - np.cos(theta[i]/2))) ** 2 + (lines[i].length - radii[i+1]*np.sin(theta[i]/2)) ** 2)
            b = np.sqrt((radii[i+1]*np.cos(theta[i]) + lines[i+1].length*np.cos(theta[i]-np.pi/2)-radii[i+1]*np.cos(theta[i]/2)) ** 2 + (radii[i+1]*np.sin(theta[i])+lines[i+1].length*np.sin(theta[i]-np.pi/2)-radii[i]*np.sin(theta[i]/2)) ** 2)
            path_length += (a+b) - (lines[i].length + lines[i+1].length)
        pl[i+1] = path_length

    return path_length

def cost_to_move(current_wp, next_wp, current_triplet, next_obstacle, distance):
    cost = 0
    cost += distance
    # cost += Point(next_obstacle.x, next_obstacle.y).distance_to(Point(current_triplet.obstacles[2].x, current_triplet.obstacles[2].y))
    return cost

def heuristic_cost(wp, obstacle):
    cost = wp.distance_to(Point(obstacle.x, obstacle.y))
    return cost

def distance_to_path(obstacle, wp1, wp2):
    return np.abs((wp2.y - wp1.y) * obstacle.x - (wp2.x - wp1.x) * obstacle.y + wp2.x * wp1.y - wp2.y * wp1.x) / wp1.distance_to(wp2)

def create_path(final_node):
    path = deque()
    current = final_node
    path.appendleft(current)
    while current.parent is not None:
        current = current.parent
        path.appendleft(current)
    return list(path)

def path_to_wp(previous_wp, wp, init_triplet, env, max_dist=2, fig=None, neighbour_func=lambda c, n, s: c.quadruplet_distance(n, s)):
    """
    This implements a simple A*-search, using the cost_to_move()
    and heuristic_cost() functions to find g and h values, respectively.
    Returns a list of triplets that the snake will move through.
    """
    plt.ion()
    wp_index = 2
    visited = [] # Closed set
    seen = []
    queue = [] # Open set
    heappush(queue, init_triplet)
    current = None
    found = False
    while queue:
        current = heappop(queue)
        visited.append(current)
        if fig is not None:
            fig.gca().clear()
            plot_desired_path([previous_wp, wp], create_path(current), env, fig)
            plt.draw()
            plt.pause(.001)
        # Check if we have reached our goal, and end if we have
        if wp.distance_to(Point(current.obstacles[2].x, current.obstacles[2].y)) < max_dist:
            found = True
            break
        possibilities = neighbour_func(current, current.obstacles[2].neighbours, env.snake_len)
        for obstacle, distance in possibilities.items():
            triplet = current.get_next_triplet(obstacle)
            if triplet in visited:
                continue
            cost = current.cost + cost_to_move(previous_wp, wp, current, obstacle, distance)
            # If the triplet is not in the queue yet, add it and update its cost and heuristic cost
            if triplet not in queue:
                seen.append(triplet)
                triplet.cost = cost
                triplet.parent = current
                triplet.h = heuristic_cost(wp, obstacle)
                heappush(queue, triplet)
            # If the new cost is better than the old one, update the old one
            else:
                for t in queue:
                    if t == triplet:
                        triplet = t
                        break
                if cost + triplet.h < triplet.cost + triplet.h:
                    triplet.cost = cost
                    triplet.parent = current
            heapify(queue)
    if not found:
        raise ValueError("No path found!")
    # Build path from parentage
    plt.ioff()
    return current

def path_finder(waypoints, radiuses, init_triplet, env, max_dist=15, neighbour_func=lambda c, n, s: c.quadruplet_distance(n, s)):
    min_dist_wp = 100000000
    index_closest_wp = 0
    start_p = Point(init_triplet.obstacles[0].x, init_triplet.obstacles[0].y)
    for i in range(len(waypoints)):
        dist = start_p.distance_to(waypoints[i])
        if(dist < min_dist_wp):
            min_dist_wp = dist
            index_closest_wp = i
            
    init_triplet=env.init_triplet
    fig = plt.figure()
    final_triplet = env.init_triplet
    final_path = [init_triplet]
    for i in range(1,len(waypoints)):
        final_triplet = path_to_wp(waypoints[i-1], waypoints[i], final_triplet, env, max_dist, fig, neighbour_func)
    return create_path(final_triplet)

def plot_visited(env, visited):
    plt.ion()
    fig = e.plot_environment(env)
    axes = fig.gca()
    for i in range(len(visited)):
        e.plot_environment(env, fig=fig)
        for obstacle in visited[i].obstacles:
            axes.add_artist(obstacle.get_circle('b'))
        plt.draw()
        input()
    plt.ioff()

if __name__ == "__main__":
    env = e.Environment(200,200,150,300, radius_func=lambda: np.random.rand() * 3 + 1, snake_len=50)
    env.save("env")
    points = create_desired_path(env,3)
    path2 = path_finder(points, None, env.init_triplet, env, 10, lambda c, n, s: c.quadruplet_distance_stupid(n, s))
    try:
        path = path_finder(points, None, env.init_triplet, env, 10)
    except ValueError:
        print("Something wrong happened")
        plot_desired_path(points, path, env=env)
        plt.show()
        pass
    print("Found path")
    # plot_visited(env, visited)
