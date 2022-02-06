from networktables import NetworkTables
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math

OUTER_LEFT_OFFSET = -3
INNER_LEFT_OFFSET = -2
INNER_RIGHT_OFFSET = 2
OUTER_RIGHT_OFFSET = 3

DEFAULT_Y = 26.5

HOOK_RADIUS = 1

NetworkTables.initialize(server='localhost')
sb = NetworkTables.getTable('SmartDashboard')

fig = plt.figure()
lines = plt.plot(
    [OUTER_LEFT_OFFSET, OUTER_LEFT_OFFSET], [0, DEFAULT_Y],
    [INNER_LEFT_OFFSET, INNER_LEFT_OFFSET], [0, DEFAULT_Y],
    [INNER_RIGHT_OFFSET, INNER_RIGHT_OFFSET], [0, DEFAULT_Y],
    [OUTER_RIGHT_OFFSET, OUTER_RIGHT_OFFSET], [0, DEFAULT_Y],
    c = 'g'
)

def generate_hook(start_x, start_y, top_angle):
    top_angle = -top_angle + 90
    
    x = np.linspace(-HOOK_RADIUS, HOOK_RADIUS, num=50, endpoint=False)
    y = np.sqrt(HOOK_RADIUS**2 - x**2)
    
    x = np.concatenate((x, x[::-1]))
    y = np.concatenate((y, -y[::-1]))
    
    angles = np.arctan2(y, x) * 180 / np.pi
    in_semicircle = abs(((angles - top_angle + 180) % 360) - 180) <= 90
    x = x[in_semicircle]
    y = y[in_semicircle]
    
    if in_semicircle[0] and in_semicircle[-1]:
        end_of_first_streak = 0
        while in_semicircle[end_of_first_streak]:
            end_of_first_streak +=1
        
        start_of_second_streak = end_of_first_streak
        while not in_semicircle[start_of_second_streak]:
            start_of_second_streak += 1
            
        x = np.concatenate((x[start_of_second_streak:], x[:end_of_first_streak]))
        y = np.concatenate((y[start_of_second_streak:], y[:end_of_first_streak]))
    
    return x + start_x + HOOK_RADIUS * math.sin(top_angle * np.pi / 180), y + start_y - HOOK_RADIUS * math.cos(top_angle * np.pi / 180)

hooks = [
    plt.plot(*generate_hook(OUTER_LEFT_OFFSET, DEFAULT_Y, 0), c='g')[0],
    plt.plot(*generate_hook(INNER_LEFT_OFFSET, DEFAULT_Y, 0), c='g')[0],
    plt.plot(*generate_hook(INNER_RIGHT_OFFSET, DEFAULT_Y, 0), c='g')[0],
    plt.plot(*generate_hook(OUTER_RIGHT_OFFSET, DEFAULT_Y, 0), c='g')[0],
]

def update(_):
    lines[0].set_data([OUTER_LEFT_OFFSET, sb.getNumber("Outer Left Climber X", 0) + OUTER_LEFT_OFFSET], [0, sb.getNumber("Outer Left Climber Y", DEFAULT_Y)])
    lines[1].set_data([INNER_LEFT_OFFSET, sb.getNumber("Inner Left Climber X", 0) + INNER_LEFT_OFFSET], [0, sb.getNumber("Inner Left Climber Y", DEFAULT_Y)])
    lines[2].set_data([INNER_RIGHT_OFFSET, sb.getNumber("Inner Right Climber X", 0) + INNER_RIGHT_OFFSET], [0, sb.getNumber("Inner Right Climber Y", DEFAULT_Y)])
    lines[3].set_data([OUTER_RIGHT_OFFSET, sb.getNumber("Outer Right Climber X", 0) + OUTER_RIGHT_OFFSET], [0, sb.getNumber("Outer Right Climber Y", DEFAULT_Y)])
    
    hooks[0].set_data(*generate_hook(OUTER_LEFT_OFFSET + sb.getNumber("Outer Left Climber X", 0), sb.getNumber("Outer Left Climber Y", DEFAULT_Y), sb.getNumber("Outer Left Climber Pivot Angle", 0)))
    hooks[1].set_data(*generate_hook(INNER_LEFT_OFFSET + sb.getNumber("Inner Left Climber X", 0), sb.getNumber("Inner Left Climber Y", DEFAULT_Y), sb.getNumber("Inner Left Climber Pivot Angle", 0)))
    hooks[2].set_data(*generate_hook(INNER_RIGHT_OFFSET + sb.getNumber("Inner Right Climber X", 0), sb.getNumber("Inner Right Climber Y", DEFAULT_Y), sb.getNumber("Inner Right Climber Pivot Angle", 0)))
    hooks[3].set_data(*generate_hook(OUTER_RIGHT_OFFSET + sb.getNumber("Outer Right Climber X", 0), sb.getNumber("Outer Right Climber Y", DEFAULT_Y), sb.getNumber("Outer Right Climber Pivot Angle", 0)))
    
    fig.gca().set(xlim=[-36, 36], ylim=[-12, 60])
    fig.gca().autoscale_view()
    return lines

animation = FuncAnimation(fig, update, interval=10)
plt.show()