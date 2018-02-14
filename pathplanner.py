import numpy as np
import environment as e
import matplotlib as plt
import math

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

    def push_front(self,ob):
        obstacles[2] = obstacles[1]
        obstacles[1] = obstacles[0]
        obstacles[0] = ob

def create_desired_path(env, num_wp):
    points = []
    start_p = Point(np.random.rand()*0.1*env.width, \
                     (np.random.rand()*0.5 +0.5)*env.height)
    end_p = Point((np.random.rand()*0.1 + 0.9 )*env.width, \
                     (np.random.rand()*0.3 + 0.5)*env.height)

    dist_x = end_p.x - start_p.x
    dist_y = end_p.y - start_p.y
    
    for i in range(num_wp) :
        p = Point(start_p.x,start_p.y)
        p.x += dist_x*i/num_wp
        p.y += dist_y*i/num_wp + np.random.rand()*dist_x/(num_wp)
        
        points.append(p)
    return points

def plot_desired_path(points):
    x = []
    y = []
    for p in points:
        x.append(p.x)
        y.append(p.y)
    plt.pyplot.plot(x,y,'r')
    plt.pyplot.xlim((0,200))
    plt.pyplot.ylim((0,200))
    plt.pyplot.show()
    


if __name__ == "__main__":
    env = e.Environment(200,50,20,50)
    points = create_desired_path(env,10)
    plot_desired_path(points)



    
    

    
