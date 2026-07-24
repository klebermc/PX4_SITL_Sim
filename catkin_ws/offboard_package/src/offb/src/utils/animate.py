# Common Python libraries
import numpy as np
import matplotlib.pyplot as plt

# plt.scatter(p[0],p[1])
# plt.scatter(wp[0],wp[1])
# # for ipb in pb:
# #     plt.scatter(ipb[0],ipb[1])
# plt.scatter(pb[0],pb[1])
# counterdz=0
# for idz in dz:
#   plt.scatter(idz[0],idz[1])
#   draw_circle(idz[0],idz[1], dzs[counterdz])
#   counterdz=counterdz+1
#
# plt.xlabel('POS X [m]')
# plt.ylabel('POS Y [m]')
# plt.axis([-3, 3, -3, 3])
# plt.grid(color='k', linestyle='-', linewidth=0.2)
# plt.draw()
# plt.pause(0.05)
# # plt.show(block=True)

class AnimateSimulation:
    def __init__(self, wp, p_hist, dz,  dzs):
        self.target = wp
        self.p_history = p_hist
        self.dz = dz
        self.dzs = dzs

    def draw_circle(self, xc,yc,r):
        angle = np.linspace( 0 , 2 * np.pi , 150 )
        radius = r
        x = radius * np.cos( angle )
        y = radius * np.sin( angle )
        # plt.plot( x+xc, y+yc )
        return (x+xc), (y+yc)

    def animate(self, num):
        if(num == len(self.p_history)-1):
            self.ax.clear()
        #PLOTS
        p = self.p_history[num,0:2]

        #self.ax.scatter(p_A[0],p_A[1],p_A[2], color = "red")
        #draw_circle(idz[0],idz[1], dzs[counterdz])
        #self.ax.scatter(p_1[0],p_1[1],p_1[2], color = "blue")

        self.p_h.set_offsets(p)
        self.p_traj_h.set_data(self.p_history[0:num,0], self.p_history[0:num,1])

        self.ptgt_h.set_offsets(self.target[0:2])

        counterdz=0
        for idz in self.dz:
            print(self.pdz_center[counterdz])
            self.pdz_center[counterdz].set_offsets(idz)
            x_dz,y_dz = self.draw_circle(idz[0],idz[1], self.dzs[counterdz])
            print(self.pdz_line[counterdz][0])
            self.pdz_line[counterdz][0].set_data(x_dz,y_dz)
            counterdz=counterdz+1

        # counterdz=0
        # for idz in dz:
          # plt.scatter(idz[0],idz[1])
          # draw_circle(idz[0],idz[1], dzs[counterdz])
          # counterdz=counterdz+1

        #self.ax.scatter(self.target[0], self.target[1], self.target[2], color = "black")
        plt.xlabel('POS X [m]')
        plt.ylabel('POS Y [m]')

        plt.grid(color='k', linestyle='-', linewidth=0.2)
        self.ax.set_aspect('equal', 'box')
        self.ax.axis(self.bounds[0:4])

        print("---->", num)
        return self.p_h,self.p_traj_h, self.pdz_center, self.pdz_line, self.ptgt_h,

    def setupAnimation(self, fig, ax, bounds):
        self.fig = fig
        self.ax = ax
        self.bounds = bounds
        self.p_h = ax.scatter(0,0, color = 'blue')
        self.p_traj_h, = ax.plot([], [], 'b-')
        self.pdz_center =[]
        self.pdz_line=[]

        counterdz=0
        for idz in self.dz:
            self.pdz_center.append(ax.scatter(0,0, color = 'red'))
            self.pdz_line.append(ax.plot([], [], 'r-'))
            counterdz=counterdz+1

        self.ptgt_h = ax.scatter(0,0, color = 'black')

        #self.sensing_h, = ax.plot([], [], 'k-')
        #self.I_star_h, = ax.plot([], [], 'b*')
        #return self.p1_h,
