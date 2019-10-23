import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
# Fixing random state for reproducibility
np.random.seed(196)
#初始数据绘图
dis = np.zeros(40)
dis2 = dis
fig, ax = plt.subplots()
line, = ax.plot(dis)
ax.set_ylim(-1, 1)
plt.grid(True)
ax.set_ylabel("distance: m")
ax.set_xlabel("time")

def update(frame):
    global dis
    global dis2
    global line
    #读入模拟
    a = np.random.rand()*2-1
    time.sleep(np.random.rand()/10)
    #绘图数据生成
    dis[0:-1] = dis2[1:]
    dis[-1] = a
    dis2 = dis
    #绘图
    line.set_ydata(dis)
    #颜色设置
    if abs(a) < 0.5:
        plt.setp(line, 'color', 'r', 'linewidth', 2.0)
    else:
        plt.setp(line, 'color', 'b', 'linewidth', 2.0)
    return line
ani = animation.FuncAnimation(fig, update,frames=None, interval=100)
plt.show()