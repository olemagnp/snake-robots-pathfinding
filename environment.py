import numpy as np
import matplotlib.pyplot as plt
import pickle
import pathplanner
from triplet import Point
from plotting import plot_environment

class Environment:
    def __init__(self, width, height, min_num_obstacles, max_num_obstacles, radius_func=lambda: np.random.rand() * 10, start_pos=(25, 25), snake_len=10, snake_width=2):
        self.width = width
        self.height = height
        start_obstacle_radius = 1
        self.obstacles = [Obstacle(start_pos[0] - 2 * start_obstacle_radius - snake_width, start_pos[1], 
                start_obstacle_radius, 0), Obstacle(start_pos[0], start_pos[1], start_obstacle_radius, 1),
                Obstacle(start_pos[0] + 2 * start_obstacle_radius + snake_width, start_pos[1], start_obstacle_radius, 2)]
        self.start_pos = start_pos
        self.init_triplet = pathplanner.Triplet(self.obstacles[:3])
        self.snake_len = snake_len
        count = 3
        while len(self.obstacles) < min_num_obstacles:
            for _ in range(max_num_obstacles - len(self.obstacles)):
                obstacle = Obstacle(width * np.random.rand(), height * np.random.rand(), radius_func(), count)
                count += 1
                collision = False
                for obs in self.obstacles:
                    if obstacle.collides_with(obs, snake_width):
                        collision = True
                        break
                if not collision:
                    self.obstacles.append(obstacle)
        
        for i in range(len(self.obstacles)):
            obstacle = self.obstacles[i]
            for obstacle2 in self.obstacles[i + 1:]:
                distance = np.sqrt((obstacle.x - obstacle2.x) ** 2 + (obstacle.y - obstacle2.y) ** 2) - obstacle.radius - obstacle2.radius
                if distance < snake_len:
                    obstacle.neighbours[obstacle2] = distance
                    obstacle2.neighbours[obstacle] = distance
    
    def get_radius(self, p, default=10):
        min_dist = float('inf')
        for ob in self.obstacles:
            d = p.distance_to(ob)
            if d < min_dist:
                min_dist = d
        return max(min_dist, default)
    
    def save(self, path):
        with open(path, 'wb') as f:
            pickle.dump(self, f)
    
    @classmethod
    def load(cls, path):
        with open(path, 'rb') as f:
            return pickle.load(f)

class Obstacle(Point):
    def __init__(self, x, y, radius, id):
        self.x = x
        self.y = y
        self.radius = radius
        self.neighbours = {}
        self.id = id
    
    def collides_with(self, other, collision_distance):
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2) < collision_distance + self.radius + other.radius
    
    def get_circle(self, color='r'):
        return plt.Circle((self.x, self.y), self.radius, color=color)
    def distance_to(self, ob):
        return np.sqrt((self.x - ob.x)**2 + (self.y - ob.y)**2)
    
    def __repr__(self):
        return "Obstacle[x=%.2f, y=%.2f, radius=%.2f]" % (self.x, self.y, self.radius)

    def __str__(self):
        return self.__repr__()

if __name__ == '__main__':
    env = Environment(200, 100, 20, 50)
    plot_environment(env)
    plt.show()
