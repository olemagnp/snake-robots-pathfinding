import numpy as np
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, width, height, min_num_obstacles, max_num_obstacles, minimum_obstacle_distance=5, radius_func=lambda: 1):
        self.width = width
        self.height = height
        self.obstacles = []
        while len(self.obstacles) < min_num_obstacles:
            for _ in range(max_num_obstacles - len(self.obstacles)):
                obstacle = Obstacle(width * np.random.rand(), height * np.random.rand(), radius_func())
                collision = False
                for obs in self.obstacles:
                    if obstacle.collides_with(obs, minimum_obstacle_distance):
                        collision = True
                        break
                if not collision:
                    self.obstacles.append(obstacle)

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius 
    
    def collides_with(self, other, collision_distance):
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2) < collision_distance + self.radius + other.radius
    
    def get_circle(self, color='r'):
        return plt.Circle((self.x, self.y), self.radius, color=color)

def plot_environment(env, snake=None):
    points = np.array([[obs.x, obs.y, (2 * obs.radius) ** 2] for obs in env.obstacles])
    axes = plt.gca()
    lim = max([env.width, env.height])
    axes.set_xlim([0, lim])
    axes.set_ylim([0, lim])
    axes.set_aspect('equal', adjustable='box')
    for obs in env.obstacles:
        axes.add_artist(obs.get_circle())
    plt.show()

if __name__ == '__main__':
    env = Environment(200, 200, 20, 50)
    plot_environment(env)
