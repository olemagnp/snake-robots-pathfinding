import numpy as np
import environment as e
import matplotlib as plt
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

def plot_desired_path(points, env=None):
    if env:
        fig = e.plot_environment(env)
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
    ax.plot(x,y,'g')
    plt.pyplot.show()


class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.distance = None

    def distance_to(self, p):
        return sqrt((self.x- p.x)^2 + (self.y-p.y)^2)
    

class Triplet:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.sides = np.array([0,0,0])
        self.cost = 0
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
        return self.sides == o.sides and self.obstacles == o.obstacles
    
    def __lt__(self, o):
        return self.cost < o.cost

    def __le__(self, o):
        return self.cost <= o.cost
    
    def quadruplet_distance(self, obs_list):
        raise NotImplementedError()

def cost_to_move(current_wp, next_wp, current_triplet, next_obstacle):
    cost = distance_to_path(next_obstacle, current_wp, next_wp)
    # cost += Point(obstacles[0].x, obstacles[0].y).distance_to(Point(next_obstacle.x, next_obstacle.y))
    return cost

def distance_to_path(obstacle, wp1, wp2):
    return np.abs((wp2.y - wp1.y) * obstacle.x - (wp2.x - wp1.x) * obstacle.y) + wp2.x * wp1.y - wp2.y * wp1.x) / wp1.distance_to(wp2)

def path_to_wp(previous_wp, wp, init_triplet, env, max_dist=2):
    visited = []
    seen = []
    queue = []
    heappush(heap, init_triplet)
    current = None
    found = False
    while queue:
        parent = current
        current = heappop(queue)
        visited.append(current)
        current.parent = parent
        if wp.distance_to(Point(current.obstacles[0].x, current.obstacles[0].y)) < max_dist:
            found = True
            break
        possibilities = current.quadruplet_distance(current.obstacles[0].neighbours)
        for obstacle in possibilities:
            triplet = current.get_next_triplet(obstacle)
            cost = current.cost + cost_to_move(previous_wp, wp, current, obstacle)
            if triplet not in seen:
                seen.append(triplet)
                triplet.cost = cost
                heappush(queue, triplet)
            else if triplet.cost < cost:
                triplet.cost = cost
                heapify(queue)
    if not found:
        raise ValueError("No path found!")
    path = deque()
    path.appendleft(current)
    while current.parent is not None:
        current = current.parent
        path.appendleft(current)
    return path


def path_finder(waypoints, radiuses, init_triplet, env):
    min_dist_wp = 100000000
    index_closest_wp = 0
    start_p = Point(init_triplet.obstacles[0].x, init_triplet.obstacles[0].y)
    for i in range(len(waypoints)):
        dist = start_p.distance_to(waypoints[i])
        if(dist < min_dist_wp):
            min_dist_wp = dist
            index_closest_wp = i
            
    final_path = init_triplet.obstacles

    for i in range(index_closest_wp,len(waypoints)):
        final_path.append(path_to_wp(waypoints[i], init_triplet, env))
        init_triplet = Triplet(final_path[-3:-1])
    return final_path

if __name__ == "__main__":
    env = e.Environment(200,200,100,150, radius_func=lambda: np.random.rand() * 3 + 1)
    points = create_desired_path(env,10)
    plot_desired_path(points, env)



    
    

    
