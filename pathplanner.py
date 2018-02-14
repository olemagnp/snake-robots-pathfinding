import numpy as np
import environment as e
import matplotlib as plt
import math

def create_desired_path(env, num_wp):
    points = []
    start_p = Point(np.random.rand()*0.1*env.width, \
                     (np.random.rand()*0.7 + 0.15)*env.height)
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

def plot_desired_path(points):
    x = []
    y = []
    for p in points:
        x.append(p.x)
        y.append(p.y)
    plt.pyplot.plot(x,y,'r')
    plt.pyplot.xlim((0,200))
    plt.pyplot.ylim((0,50))
    plt.pyplot.show()


class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def distance_to(self, p):
        dist = sqrt((self.x- p.x)^2 + (self.y-p.y)^2)
    

class Triplet:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.sides = np.array([0,0,0])
        self.distance_to_next=None

    def push_front(self,ob):
        self.obstacles[2] = self.obstacles[1]
        self.obstacles[1] = self.obstacles[0]
        self.obstacles[0] = ob
    
    def quadruplet_distance(self):
        r = self.obstacles[1].radius


def path_finder(waypoints, init_triplet, env):
    min_dist_wp = 100000000
    index_closest_wp = 0
    start_p = Point(init_triplet.obstacles[0].x, init_triplet.obstacles[0].y)
    for i in range(len(waypoints)):
        dist = start_p.distance_to(waypoints[i])
        if(dist < min_dist_wp):
            min_dist_wp = dist
            index_closest_wp = i
            
    for i in range(index_closest_wp,len(waypoints)):
        
        
    


if __name__ == "__main__":
    env = e.Environment(200,50,20,50)
    points = create_desired_path(env,10)
    plot_desired_path(points)



    
    

    
