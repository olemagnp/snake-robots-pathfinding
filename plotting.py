import numpy as np
from triplet import Point
import matplotlib.pyplot as plt

def plot_environment(env, snake=None, fig=None):
    fig = fig if fig else plt.figure()
    ax = fig.gca()
    points = np.array([[obs.x, obs.y, (2 * obs.radius) ** 2] for obs in env.obstacles])
    axes = fig.add_subplot(111)
    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])
    axes.set_aspect('equal', adjustable='box')
    for obs in env.obstacles:
        axes.add_artist(obs.get_circle())
    return fig

def circle_coord(prevPos, obstacle, side):
    # This should give the position on the side of the obstacle, but something is wrong
    dx = obstacle.x - prevPos.x
    dy = obstacle.y - prevPos.y
    dd = np.sqrt(dx ** 2 + dy ** 2)
    opposite = obstacle.radius
    a = np.arcsin(obstacle.radius / dd)
    angle = np.arctan2(dy, dx)
    if side == 1:
        x = obstacle.x + obstacle.radius * np.sin(a - np.sign(dx) * angle)
        y = obstacle.y + obstacle.radius * np.cos(a - np.sign(dy) * angle)
    else:
        x = obstacle.x + obstacle.radius * np.sin(a + np.sign(dx) * angle)
        y = obstacle.y + obstacle.radius * np.cos(a + np.sign(dy) * angle)
    return Point(x, y)

def plot_path(path=None, fig=None):
    fig = plt.figure() if fig is None else fig
    axes=fig.gca()
    if path is not None:
        path = list(path)
        path_x = []
        path_y = []
        for obstacle in path[0].obstacles:
            axes.add_artist(obstacle.get_circle(color='b'))
            path_x.append(obstacle.x)
            path_y.append(obstacle.y)
        for triplet in path[1:]:
            axes.add_artist(triplet.obstacles[2].get_circle(color='b'))
            path_x.append(triplet.obstacles[2].x)
            path_y.append(triplet.obstacles[2].y)
        axes.plot(path_x, path_y, 'b')
        axes.plot(path_x, path_y, 'g+')
    return fig

def plot_desired_path(points, color, radii, fig=None):
    # Plot the environment and use its figure to plot the desired path
    fig = plt.figure() if fig is None else fig
    fig = plot_waypoints(points, color, radii, fig)
    return fig
    
def plot_waypoints(points, color, radii, fig=None):
    fig = plt.figure() if fig is None else fig
    x = []
    y = []
    ax = fig.gca()
    for i, p in enumerate(points):
        x.append(p.x)
        y.append(p.y)
        circle = plt.Circle((p.x, p.y), radii[i], fill=False, color=color)
        ax.add_artist(circle)
        plt.draw()
    ax.plot(x,y, color+'o-')
    return fig

def plot_active_segment(previous_wp, wp, color, radii, path=None, fig=None):
    # Plot the environment and use its figure to plot the desired path
    fig = plt.figure() if fig is None else fig
    fig = plot_waypoints([previous_wp, wp], color, radii, fig)
    fig = plot_path(path, fig)
    return fig