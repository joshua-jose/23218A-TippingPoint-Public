import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig, axs = plt.subplots(2)
ax1 = axs[0]
ax2 = axs[1]


def animate(i):
    with open('graph.txt', 'r') as f:
        graph_data = f.read()

    lines = graph_data.split('\n')
    xs = []
    ys = []
    zs = []
    rvs = []
    kvs = []

    points = []
    for line in lines[-500:]:
        if len(line) > 1 and "run" in line:
            points = []
        if len(line) > 1 and "graph:" in line:
            data = line.replace("graph:", "").replace("\n", "")
            points.append([float(i) for i in data.split(',')])

    for i in points:
        xs.append(float(i[0])/1000)
        ys.append(float(i[1]))
        zs.append(float(i[2]))

    ax1.clear()
    ax2.clear()

    ax1.plot(xs, ys, 'r')
    ax2.plot(xs, zs, 'b')

    #ax2.plot(xs, rvs, 'rx')
    #ax2.plot(xs, kvs, 'b')


ani = animation.FuncAnimation(fig, animate, interval=100)
# animate(1)
plt.show()
