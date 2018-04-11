import numpy as np

class Triplet:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.side = 0
        self.cost = 0 # Actual cost to move to this obstacle 
        self.h = 0 # Heuristic cost left to the goal
        self.parent = None
        self.deadend = []
        self.is_first_in_path = False

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
    
    def __hash__(self):
        return self.__repr__().__hash__()
    
    def __lt__(self, o):
        return self.cost + self.h < o.cost + o.h

    def __le__(self, o):
        return self.cost + self.h <= o.cost + o.h
        
    
    def __repr__(self):
        return "Triplet" + str((self.obstacles[0].id, self.obstacles[1].id, self.obstacles[2].id))

    def __str__(self):
        return self.__repr__()

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

        cur_triplet_dist = Triplet.triplet_distance([tangent_lines[0][(self.side+1)%2], tangent_lines[1][self.side]], self.obstacles[1].radius, self.side)
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

            lines = [tangent_lines[1][self.side], tangent_lines_neigh[(self.side+1)%2]]
            radius = self.obstacles[2].radius
            obstacle_distance = Line.added_distance(lines, radius, self.side)

            if cur_triplet_dist + obstacle_distance < snake_length:
                ob_dict[ob] = obstacle_distance
        return ob_dict  # dictionary of {Obstacle: required snake length}

    @classmethod
    def triplet_distance(cls, lines, radius, s):
        path_length = 0
        for line in lines:
            path_length += line.length

        theta = lines[0].angle_to_line(lines[1])
        if (theta <= 0) == ((s % 2) == 0):  # OUTSIDE
            path_length += radius*abs(theta)
        else:  # INSIDE
            theta = abs(theta)
            # This code finds a point on the middle of the arch between the points where the two tangent lines intersects with the circle.
            # Then it finds the distances from the endpoints to the point on the arch, and adds these to the path_length. Then it subtracts the length of the tangent lines from
            # path_length, as these are added earlier in the code. (See picture on google drive)
            a = np.sqrt((radius * (1 - np.cos(theta/2))) ** 2 + (lines[0].length - radius*np.sin(theta/2)) ** 2)
            b = np.sqrt((radius*np.cos(theta) + lines[1].length*np.cos(theta-np.pi/2)-radius*np.cos(theta/2)) ** 2 + (radius*np.sin(theta)+lines[1].length*np.sin(theta-np.pi/2)-radius*np.sin(theta/2)) ** 2)
            path_length += (a+b) - (lines[0].length + lines[1].length)

        return path_length

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

    @classmethod
    def added_distance(cls, lines, radius, s):
        path_length = lines[-1].length

        theta = lines[0].angle_to_line(lines[1])
        if (theta <= 0) == ((s % 2) == 0):  # OUTSIDE
            path_length += radius*abs(theta)
        else:  # INSIDE
            theta = abs(theta)
            # This code finds a point on the middle of the arch between the points where the two tangent lines intersects with the circle.
            # Then it finds the distances from the endpoints to the point on the arch, and adds these to the path_length. Then it subtracts the length of the tangent lines from
            # path_length, as these are added earlier in the code. (See picture on google drive)
            a = np.sqrt((radius * (1 - np.cos(theta/2))) ** 2 + (lines[0].length - radius*np.sin(theta/2)) ** 2)
            b = np.sqrt((radius*np.cos(theta) + lines[1].length*np.cos(theta-np.pi/2)-radius*np.cos(theta/2)) ** 2 + (radius*np.sin(theta)+lines[1].length*np.sin(theta-np.pi/2)-radius*np.sin(theta/2)) ** 2)
            path_length += (a+b) - (lines[0].length + lines[1].length)

        return path_length