import numpy as np
import environment as e
import matplotlib.pyplot as plt
import math
from collections import deque
from heapq import *
import time
from sys import exit
from triplet import Triplet, Point, Line
from plotting import plot_desired_path, plot_active_segment

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

### Cost calculation ###
def cost_to_move(current_wp, next_wp, current_triplet, next_obstacle, distance):
    cost = 0
    cost += distance
    # cost += Point(next_obstacle.x, next_obstacle.y).distance_to(Point(current_triplet.obstacles[2].x, current_triplet.obstacles[2].y))
    return cost

def heuristic_cost(wp, obstacle):
    cost = wp.distance_to(Point(obstacle.x, obstacle.y)) - obstacle.radius
    return cost

def distance_to_path(obstacle, wp1, wp2):
    return np.abs((wp2.y - wp1.y) * obstacle.x - (wp2.x - wp1.x) * obstacle.y + wp2.x * wp1.y - wp2.y * wp1.x) / wp1.distance_to(wp2)

### Unknown environment stuff ###
def filter_queue(queue, newActive, prevActive):
    newqueue = []  # Open set
    while queue:
        triplet = heappop(queue)
        current = triplet
        "Follow the triplets parents until you reach the previously active node"
        print("Previous active:", prevActive)
        try:
            # TODO current sometimes becomes init_triplet
            print("Current:", current)
            print("Par:", current.parent.obstacles[2])
            while current.parent is not None and current.parent.obstacles[2] != prevActive:
                current = current.parent

        except AttributeError:
            print("Attribute error in filter_queue")
            print("Current: " + str(current))
            print("Parent: " + str(current.parent))
            exit(1)
        "Is the new active in the path?"
        if current.obstacles[2] == newActive:
            heappush(newqueue, triplet)
    return newqueue


def new_active(endTriplet, activeObstacle):
    """"
    Returns the second obstacle in the path from activeObstacle to endObstacle
    """
    current = endTriplet
    while current.parent.obstacles[2] != activeObstacle:
        current = current.parent
    return current

### More or less standard A* ###
def create_path(final_node):
    path = deque()
    current = final_node
    path.appendleft(current)
    while current.parent is not None:
        current = current.parent
        path.appendleft(current)
    return list(path)

def path_finder(waypoints, radiuses, init_triplet, env, max_dist=15, neighbour_func=lambda c, n, s: c.quadruplet_distance(n, s), acceptance_radiuses=None):
    min_dist_wp = 100000000
    index_closest_wp = 0
    start_p = Point(init_triplet.obstacles[0].x, init_triplet.obstacles[0].y)
    if acceptance_radiuses is None:
        acceptance_radiuses = []
        for i in range(len(waypoints)):
            dist = start_p.distance_to(waypoints[i])
            acceptance_radiuses.append(env.get_radius(waypoints[i], max_dist))
            if(dist < min_dist_wp):
                min_dist_wp = dist
                index_closest_wp = i
    init_triplet=env.init_triplet
   
    final_triplet = env.init_triplet
    final_path = [init_triplet]
    fig = plt.figure()
    deadends = {}
    for i in range(1,len(waypoints)):
        final_triplet = path_to_wp(waypoints, i, final_triplet, env, acceptance_radiuses, fig, neighbour_func, deadends=deadends)
        if final_triplet is None:
            return None
    return create_path(final_triplet)

def path_to_wp(waypoints, wp_index, init_triplet, env, wp_radii, fig=None, neighbour_func=lambda c, n, s: c.quadruplet_distance(n, s), scope_range = 50, deadends=None):
    """
    This implements a simple A*-search, using the cost_to_move()
    and heuristic_cost() functions to find g and h values, respectively.
    Returns a list of triplets that the snake will move through.

    TODO:
    Catch when it is unable to complete and physically backtrack, marking the current path unfeasible
    Investigate why some triplets have no parents
    tests
    """
    plt.ion()
    deadends = deadends if deadends else {}
    previous_wp = waypoints[wp_index-1]
    wp = waypoints[wp_index]
    max_dist=wp_radii[wp_index]
    visited = [] # Closed set
    seen = []
    queue = [] # Open set
    heappush(queue, init_triplet)
    current = None
    found = False
    active_triplet = init_triplet
    init_triplet.is_first_in_path = True
    print(active_triplet)
    active = init_triplet.obstacles[2]
    iteration = 1
    wps_left = 0
    while True:
        while queue:
            current = heappop(queue)
            visited.append(current)
            if fig is not None:
                ax = fig.gca()
                ax.clear()
                fig = e.plot_environment(env, fig=fig)
                fig = plot_desired_path(waypoints, 'g', wp_radii, fig=fig)
                fig = plot_active_segment(previous_wp, wp, 'm', wp_radii, path=create_path(current), fig=fig)                
                circle = plt.Circle((active.x, active.y), 4, color='black')
                circle2 = plt.Circle((active.x, active.y), scope_range, fill=False,  color='yellow')
                ax.add_artist(circle)
                ax.add_artist(circle2)
                plt.draw()
                plt.pause(.001)
            # Check if we have reached our goal, and end if we have
            if wp.distance_to(Point(current.obstacles[2].x, current.obstacles[2].y)) <= max_dist:
                found = True
                while wps_left > 0:
                    current = path_to_wp(waypoints, wp_index + 1, current, env, wp_radii, fig, neighbour_func, scope_range, deadends)
                    wps_left -= 1
                break
            
            if active.distance_to(current.obstacles[2]) > scope_range:
                previous_active = active
                active_triplet = new_active(current, active)
                active = active_triplet.obstacles[2]
                queue = filter_queue(queue, active, previous_active)
            
            add_neighbors_to_queue(init_triplet, current, queue, visited, seen,previous_wp, wp, neighbour_func, deadends)
        
        if not found:
            print("No path found, backtracking")
            if active_triplet.parent is None:
                return None
            if active.id not in active_triplet.parent.deadend:
                deadends[active_triplet.parent] = deadends.get(active_triplet.parent, []) + [active.id]
            print("Active id: ", active.id)
            print("Current parent deadend = ", deadends.get(active_triplet.parent, []))
            if active_triplet.is_first_in_path:
                active_triplet.is_first_in_path = False
                wp_index -= 1
                previous_wp = waypoints[wp_index - 1]
                wp = waypoints[wp_index]
                wps_left += 1
            active_triplet = current.parent
            active = active_triplet.obstacles[2]
            current = active_triplet
            add_neighbors_to_queue(init_triplet, current, queue, visited, seen, previous_wp, wp, neighbour_func, deadends)
            continue
        break

    plt.ioff()
    return current

def add_neighbors_to_queue(init_triplet, current, queue, visited, seen,previous_wp, wp, neighbour_func, deadends):
    possibilities = neighbour_func(current, current.obstacles[2].neighbours, env.snake_len)
    for obstacle, distance in possibilities.items():
        #if current !=init_triplet:
            #print("Current deadend = ", current.parent.deadend, " Current obstacle id: ", obstacle.id)
        #
        if obstacle.id in deadends.get(current, []):
            print("Obstacle id: ", obstacle.id)
            print("Not a good path")
            continue
        else:
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
    return 

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
    # env = e.Environment(200,200,100,150, radius_func=lambda: np.random.rand() * 3 + 1, snake_len=50)
    # env.save("env")
    env = e.Environment.load("env")
    points = create_desired_path(env,3)
    path2 = path_finder(points, None, env.init_triplet, env, 30, lambda c, n, s: c.quadruplet_distance_stupid(n, s))
    """
    try:
        path = path_finder(points, None, env.init_triplet, env, 10)
    except ValueError:
        print("Something wrong happened")
        plot_desired_path(points, path, env=env)
        plt.show()
        pass
    """
    if path2 is not None:
        print("Found path")
    else:
        print("No path found")
    input()
    # plot_visited(env, visited)
