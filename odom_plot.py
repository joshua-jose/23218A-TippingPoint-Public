import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import matplotlib.image as mpimg
import math

style.use('ggplot')

fig, ax = plt.subplots()

for spine in ax.spines.values():
    spine.set_visible(False)

plt.gca().set_aspect('equal')

field = mpimg.imread('VRCField.png')

arrow = None
dot = None


def draw_robot(x, y, theta, ax):
    global arrow, dot
    robot_size = 1

    x1 = x - (robot_size/2) * math.cos(theta)
    dx = robot_size * math.cos(theta)
    y1 = y - (robot_size/2) * math.sin(theta)
    dy = robot_size * math.sin(theta)

    if arrow is not None:
        arrow.remove()
    if dot is not None:
        dot.pop(0).remove()

    arrow = ax.arrow(x1, y1, dx, dy, color="r", width=0.07,
                     length_includes_head=False)
    dot = ax.plot(x, y, 'ro')


ax.set_xlim([0, 12])
ax.set_ylim([0, 12])
plt.imshow(field, extent=(0, 12, 12, 0))

#x, y, theta = 0, 0, 0


def animate(i):
    with open('graph.txt', 'r') as f:
        graph_data = f.read()

    lines = graph_data.split('\n')
    x, y, theta = 0, 0, 0

    points = []
    for line in reversed(lines):
        if len(line) > 1 and "o:" in line:
            data = line.replace("o:", "").replace("\n", "")
            x, y, theta = [float(i) for i in data.split(',')]
            break

    draw_robot(x, y, -theta + math.pi/2, ax)


'''
def mouse_move(event):
    global x, y
    if event.xdata is not None and event.ydata is not None:
        x, y = event.xdata, event.ydata


plt.connect('motion_notify_event', mouse_move)
'''

ani = animation.FuncAnimation(fig, animate, interval=75)
# animate(1)
plt.show()
