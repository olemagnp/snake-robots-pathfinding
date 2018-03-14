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

def plot_desired_path(points, path=None, env=None):
    # Plot the environment and use its figure to plot the desired path
    if env:
        fig = e.plot_environment(env, path=path)
        ax = fig.gca()
        ax.set_xlim([0, env.width])
        ax.set_ylim([0, env.height])
    else:
        fig = plt.figure()
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
    

class Triplet:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.sides = 0
        self.cost = 0 # Actual cost to move to this obstacle 
        self.h = 0 # Heuristic cost left to the goal
        self.parent = None

    def push_front(self,ob):
        self.obstacles[2] = self.obstacles[1]
        self.obstacles[1] = self.obstacles[0]
        self.obstacles[0] = ob
    
    def get_next_triplet(self, ob):
        trip = Triplet([ob] + self.obstacles[:-1])
        trip.sides = (self.sides + 1) % 2
        return trip
    
    def __eq__(self, o):
        return self.sides == o.sides and np.all(self.obstacles == o.obstacles)
    
    def __lt__(self, o):
        return self.cost + self.h < o.cost + o.h

    def __le__(self, o):
        return self.cost + self.h <= o.cost + o.h
    
    def __str__(self):
        return "Triplet[" + str(self.obstacles[0]) + ", " + str(self.obstacles[1]) + ", " + str(self.obstacles[2]) + "]"
    
    def quadruplet_distance(self, obs_dict, snake_len):
        """
        Return a list of obstacles that the snake can move to, given the snake length
        and dictionary of obstacles -> distance
        """
        point0 = Point(self.obstacles[0].x, self.obstacles[0].y)
        point1 = Point(self.obstacles[1].x, self.obstacles[1].y)
        point2 = Point(self.obstacles[2].x, self.obstacles[2].y)

        snake_left = snake_len - point0.distance_to(point1) - point1.distance_to(point2)
        reachable_pts = {}
        for obstacle, distance in obs_dict.items():
            if snake_left >= distance:
                reachable_pts[obstacle] = distance
        return reachable_pts


def cost_to_move(current_wp, next_wp, current_triplet, next_obstacle, distance):
    cost = 0
    cost += distance
    return cost

def heuristic_cost(wp, obstacle):
    cost = wp.distance_to(Point(obstacle.x, obstacle.y))
    return cost

def distance_to_path(obstacle, wp1, wp2):
    return np.abs((wp2.y - wp1.y) * obstacle.x - (wp2.x - wp1.x) * obstacle.y + wp2.x * wp1.y - wp2.y * wp1.x) / wp1.distance_to(wp2)

def filter_queue(queue, newActive, prevActive):
    newqueue = []  # Open set
    while queue:
        triplet = heappop(queue)
        parent = triplet
        "Follow the triplets parents until you reach the previously active node"
        while parent.parent.obstacles[0] != prevActive:
            print(parent.parent.obstacles[0].id)
            parent = parent.parent
        "Is the new active in the path?"
        if parent.obstacles[0] == newActive:
            heappush(newqueue, triplet)

    "Should this be run every pop?"
    heapify(newqueue)
    return newqueue


def new_active(endTriplet, activeObstacle):
    """"
    Returns the second obstacle in the path from activeObstacle to endObstacle
    """
    print("entered")
    parent = endTriplet
    while parent.parent.obstacles[0] != activeObstacle:
        print(parent.parent.obstacles[0].id)
        parent = parent.parent
    print("exited")
    return parent.obstacles[0]

def path_to_wp(previous_wp, wp, init_triplet, env, max_dist=2, scope_range=70):
    """
    This implements a simple A*-search, using the cost_to_move()
    and heuristic_cost() functions to find g and h values, respectively.
    Returns a list of triplets that the snake will move through.

    TODO:
    Catch when it is unable to complete and physically backtrack, marking the current path unfeasible
    Find a way to correctly backtrack in new_active and filter_queue
    tests
    """
    wp_index = 2
    visited = [] # Closed set
    seen = []
    queue = [] # Open set
    heappush(queue, init_triplet)
    current = None
    found = False
    active = init_triplet.obstacles[0]
    while queue:
        parent = current
        current = heappop(queue)
        visited.append(current)
        current.parent = parent
        # Check if we have reached our goal, and end if we have
        if wp.distance_to(Point(current.obstacles[0].x, current.obstacles[0].y)) < max_dist:
            found = True
            break
        "should it be current.obstacles[2]?"
        if active.distance_to(current.obstacles[0]) > scope_range:
            previous_active = active
            active = new_active(current, active)
            queue = filter_queue(queue, active, previous_active)

        possibilities = current.quadruplet_distance(current.obstacles[0].neighbours, env.snake_len)
        for obstacle, distance in possibilities.items():
            triplet = current.get_next_triplet(obstacle)
            if triplet in visited:
                continue
            cost = current.cost + cost_to_move(previous_wp, wp, current, obstacle, distance)
            # If the triplet is not in the queue yet, add it and update its cost and heuristic cost
            if triplet not in queue:
                seen.append(triplet)
                triplet.cost = cost
                triplet.h = heuristic_cost(wp, obstacle)
                heappush(queue, triplet)
            # If the new cost is better than the old one, update the old one
            else:
                if cost + triplet.h < triplet.cost + triplet.h:
                    triplet.cost = cost
            heapify(queue)
    if not found:
        raise ValueError("No path found!")
    # Build path from parentage
    path = deque()
    path.appendleft(current)
    while current.parent is not None:
        current = current.parent
        path.appendleft(current)
    return list(path)

def path_finder(waypoints, radiuses, init_triplet, env, max_dist=15):
    # min_dist_wp = 100000000
    # index_closest_wp = 0
    # start_p = Point(init_triplet.obstacles[0].x, init_triplet.obstacles[0].y)
    # for i in range(len(waypoints)):
    #     dist = start_p.distance_to(waypoints[i])
    #     if(dist < min_dist_wp):
    #         min_dist_wp = dist
    #         index_closest_wp = i
            
    final_path = []
    init_triplet=env.init_triplet
    for i in range(1,len(waypoints)):
        print(i)
        print(init_triplet)
        final_path.extend(path_to_wp(waypoints[i-1], waypoints[i], init_triplet, env, max_dist))
        init_triplet = final_path[-1]
    return final_path

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
    env = e.Environment(200,200,200,500, radius_func=lambda: np.random.rand() * 3 + 1, snake_len=50)
    points = create_desired_path(env,3)
    path = path_finder(points, None, env.init_triplet, env, 15)
    plot_desired_path(points, path, env)
    print("Found path")
    # plot_visited(env, visited)
