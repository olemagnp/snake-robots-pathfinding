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
        self.side = 0                   ### side = 0 - snake is right of object[2].
                                        ### side = 1 - snake is left of object[2].
        self.distance_to_next=None

    def push_front(self, ob):
        self.obstacles[2] = self.obstacles[1]
        self.obstacles[1] = self.obstacles[0]
        self.obstacles[0] = ob
    
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


def length_of_three_lines(lines, radii, s):  # s is side
    path_length = 0
    for line in lines:
        path_length += line.length

    pl = [path_length, 0, 0]

    theta = [lines[0].angle_to_line(lines[1]), lines[1].angle_to_line(lines[2])]

    for i in range(2):

        if (theta[i] <= 0) == (((s+i) % 2) == 0):  # OUTSIDE
            print("outside")
            path_length += radii[i+1]*abs(theta[i])
        else:  # INSIDE
            print("inside")
            # This code finds a point on the middle of the arch between the points where the two tangent lines intersects with the circle.
            # Then it finds the distances from the endpoints to the point on the arch, and adds these to the path_length. Then it subtracts the length of the tangent lines from
            # path_length, as these are added earlier in the code. (See picture on google drive)
            path_length += np.sqrt((radii[i+1] * (1 - np.cos(theta[i]/2))) ** 2 + (lines[i].length - radii[i+1]*np.sin(theta[i]/2)) ** 2)
            print(np.sqrt((radii[i+1] * (1 - np.cos(theta[i] / 2))) ** 2 + (lines[i].length - radii[i+1] * np.sin(theta[i] / 2)) ** 2))
            path_length += np.sqrt((radii[i+1]*np.cos(theta[i]) + lines[i+1].length*np.cos(theta[i]-np.pi/2)-radii[i+1]*np.cos(theta[i]/2)) ** 2 + (radii[i+1]*np.sin(theta[i]+lines[i+1].length*np.sin(theta[i]-np.pi/2)-radii[i]*np.sin(theta[i]/2)) ** 2))
            print(np.sqrt((radii[i+1]*np.cos(theta[i]) + lines[i+1].length*np.cos(theta[i]-np.pi/2)-radii[i+1]*np.cos(theta[i]/2)) ** 2 + (radii[i+1]*np.sin(theta[i]+lines[i+1].length*np.sin(theta[i]-np.pi/2)-radii[i]*np.sin(theta[i]/2)) ** 2)))
            path_length -= (lines[i].length + lines[i+1].length)
        pl[i+1] = path_length

    print("Path lengths:", pl)

    return path_length


def path_finder(waypoints, init_triplet, env):
    min_dist_wp = 100000000
    index_closest_wp = 0
    start_p = Point(init_triplet.obstacles[0].x, init_triplet.obstacles[0].y)
    for i in range(len(waypoints)):
        dist = start_p.distance_to(waypoints[i])
        if(dist < min_dist_wp):
            min_dist_wp = dist
            index_closest_wp = i
            
    for i in range(index_closest_wp, len(waypoints)):
        continue

if __name__ == "__main__":
    env = e.Environment(200, 200, 100, 150)
    # e.plot_environment(env)

    points = create_desired_path(env, 10)
    # plot_desired_path(points)

    ob1 = e.Obstacle(10, 11, 1)
    ob2 = e.Obstacle(15.707, 9.293, 1)
    ob3 = e.Obstacle(14.293, 15.707, 1)
    trip = Triplet([ob1, ob2, ob3])
    ob4 = e.Obstacle(20, 14, 1)

    print(trip.quadruplet_distance([ob4], 70))

    """
    trip = Triplet(env.obstacles[0:3])
    trip.side = 0

    print(trip.quadruplet_distance(env.obstacles[3].neighbours, 50))

    trip.side = 1

    print(trip.quadruplet_distance(env.obstacles[3].neighbours, 50))
    """
