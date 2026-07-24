import numpy as np
# import matplotlib.pyplot as plt
from enum import Enum

# Olfati-Saber-style flocking controller: combines a "gamma-agent" term that
# pulls the vehicle toward a target setpoint with a "beta-agent" repulsive
# term that pushes it away from danger zones (DZ), using the standard
# sigma-norm / bump-function (rho_h) smoothing from that framework.
class Controller:
    # def __init__(self, target, v_A, lethality_radius):

    def done(self, POS, Setpoint):
        done = self.norm(POS-Setpoint) < 0.25
        return done

    # POS/VEL: current vehicle position/velocity.
    # Setpoint: target position to fly toward (gamma term).
    # DZ_Positions/DZ_Size: danger-zone centers and radii to avoid (beta term).
    # DistAvoidDZEZ: distance at which danger-zone avoidance starts engaging.
    # EndGameTrigger: when set, disables danger-zone avoidance entirely
    # (e.g. final approach, where avoidance maneuvers are no longer wanted).
    def flocking_based_controller(self, POS, VEL, Setpoint, DZ_Positions, DZ_Size, DistAvoidDZEZ, EndGameTrigger):
        c_gamma_1=0.5
        c_gamma_2=5.0
        c_beta_1=2
        c_beta_2=2.25

        epslon=0.5;
        n=3;

        TgtVEL   = np.array([0.0,0.0,0.0]);
        u_gamma  = np.array([0.0,0.0,0.0]);
        u_beta   = np.array([0.0,0.0,0.0]);
        pos_beta = np.array([0.0,0.0,0.0]);

        DZ_Positions=np.array(DZ_Positions, ndmin=2);

        pos_beta[0]=Setpoint[0]
        pos_beta[1]=Setpoint[1]


        QuantityDZ = DZ_Positions.shape[0];

        for i in range(0,QuantityDZ):
            DZ_Position = np.array(DZ_Positions[i]);
            DZ_Position[2]=POS[2]; # THIS WAY I WILL AVOID THE DZ REGARDLESS THE HEIGHT

            # ------------- Danger Zone  ----------
            sigma_d_obs=self.sigma_norm(DZ_Size[i]);
            int_range_r_obs=DistAvoidDZEZ;
            sigma_int_range_obs=self.sigma_norm(int_range_r_obs);
            # print(sigma_d_obs,int_range_r_obs,sigma_int_range_obs)


            #To run simulation faster,
            #avoid the calculations if the DZ/EZ is out of range
            if self.norm(POS - DZ_Position)<int_range_r_obs+DZ_Size[i]:
                print(" dz{}={:.3f}".format(int(i),self.norm(POS - DZ_Position)-DZ_Size[i]));
                mu = DZ_Size[i]/self.norm(POS - DZ_Position);      # obtain the vector from agent to object
                a_k = np.array((POS - DZ_Position)/self.norm(POS - DZ_Position));
                P=np.identity(n) - a_k @ (a_k.T);                   #matrix multiply
                pos_beta=mu*POS+(1-mu)*DZ_Position;         #position of beta agent (#olfati-saber modeling)
                # print(pos_beta,P,np.identity(n), a_k , a_k.T)

                vel_beta=mu*(P@VEL);                         #velocity of beta agent (#olfati-saber modeling)
                v=vel_beta/self.norm(vel_beta);
                v=v*self.norm(VEL);
                # print(VEL, vel_beta,  v);

                diff=pos_beta-POS;
                sigma_diff=self.sigma_norm(diff);
                sigma1=self.sigma_1(sigma_diff-sigma_d_obs);
                rho=self.rho_h(sigma_diff/sigma_int_range_obs);

                phi_beta = rho*(sigma1-1);
                nij = diff/np.sqrt(1+epslon*(pow(self.norm(diff),2)));
                b_ik = rho;

                pos_comp = c_beta_1*phi_beta*nij;
                vel_comp = c_beta_2*b_ik*(vel_beta-VEL);

                u_beta = u_beta + vel_comp + pos_comp ;
                # print(self.norm(pos_comp), self.norm(vel_comp))

                u_beta[2]=0;

                # pos_beta_all=np.concatenate((pos_beta_all,pos_beta),axis=0)

        if EndGameTrigger==1:
            #Turning off Danger Zone avoidance term
            u_beta=u_beta*0;

        # -------- TARGET controller --------
        u_gamma[0] = c_gamma_1*(Setpoint[0]-POS[0]) + c_gamma_2*(TgtVEL[0]-VEL[0])
        u_gamma[1] = c_gamma_1*(Setpoint[1]-POS[1]) + c_gamma_2*(TgtVEL[1]-VEL[1])
        # outSigma1 = self.sigma_1(Setpoint[0:2]-POS[0:2])
        # outNotSigma1=(Setpoint[0:2]-POS[0:2])
        # print(outSigma1,outNotSigma1)
        # u_gamma[0] = c_gamma_1*(outSigma1[0]) + c_gamma_2*(TgtVEL[0]-VEL[0])
        # u_gamma[1] = c_gamma_1*(outSigma1[1]) + c_gamma_2*(TgtVEL[1]-VEL[1])

        AccCmdE= u_gamma + u_beta

        return AccCmdE , u_gamma , u_beta, pos_beta

    # sigma_norm/sigma_1/rho_h are the standard smooth, bounded-gradient
    # functions from Olfati-Saber's flocking framework: sigma_norm gives a
    # differentiable stand-in for Euclidean distance, sigma_1 a saturated
    # unit-vector-like function, and rho_h ("bump function") smoothly fades
    # a term from 1 to 0 as its input goes from 0 to 1 (used above to ramp
    # danger-zone avoidance in/out as the vehicle approaches DistAvoidDZEZ).
    def sigma_norm(self,input):
        epslon=0.5;
        return (1/epslon)*(np.sqrt(1+epslon*(pow(self.norm(input),2)))-1);

    def sigma_1(self,input):
        return input/(np.sqrt(1+pow(self.norm(input),2)));

    def rho_h(self,input):
        if input >= 0 and input < 0.5:
            rho=1;
        elif input >= 0.5 and input<1:
            rho= 0.5 * (1 + np.cos( np.pi * ((input-0.5)/(1-0.5))));
        else :
            rho=0;
        return rho;

    def normAB(self,a,b):
        return np.linalg.norm(a - b)

    def norm(self,a):
        return np.linalg.norm(a)

    def saturate(self,val, Vmin, Vmax):
        return np.max([np.min([val,Vmax]),Vmin])

#
#
#
# class AnimateSimulation:
#     def __init__(self, target, p_1_history,p_A_history, I_star_history,  sensing_range):
#         self.target = target
#         self.sensing_range = sensing_range
#         self.p_1_history = p_1_history
#         self.p_A_history = p_A_history
#         self.I_star_history = I_star_history
#
#
#     def animate(self, num):
#         if(num == len(self.p_1_history)-1):
#             self.ax.clear()
#         #PLOTS
#         p_1 = self.p_1_history[num][0:2]
#         p_A = self.p_A_history[num][0:2]
#         #self.ax.scatter(p_A[0],p_A[1],p_A[2], color = "red")
#         #draw_circle(idz[0],idz[1], dzs[counterdz])
#         #self.ax.scatter(p_1[0],p_1[1],p_1[2], color = "blue")
#         self.p1_h.set_offsets(p_1)
#         self.p1_traj_h.set_data(Extract(self.p_1_history, 0)[0:num], Extract(self.p_1_history, 1)[0:num])
#
#         self.pA_h.set_offsets(p_A)
#         self.pA_traj_h.set_data(Extract(self.p_A_history, 0)[0:num], Extract(self.p_A_history, 1)[0:num])
#
#         self.ptgt_h.set_offsets(self.target[0:2])
#
#         th = np.linspace(0, 2*np.pi, 100)
#         x_sens = self.sensing_range*np.cos(th) + self.target[0]
#         y_sens = self.sensing_range*np.sin(th) + self.target[1]
#
#         self.sensing_h.set_data(x_sens,y_sens)
#
#         self.I_star_h.set_data(self.I_star_history[num][0],self.I_star_history[num][1])
#
#         #self.ax.scatter(self.target[0], self.target[1], self.target[2], color = "black")
#         plt.xlabel('POS X [m]')
#         plt.ylabel('POS Y [m]')
#
#         plt.grid(color='k', linestyle='-', linewidth=0.2)
#         self.ax.set_aspect('equal', 'box')
#         self.ax.axis(self.bounds[0:4])
#
#         return self.p1_h,self.p1_traj_h, self.pA_h, self.pA_traj_h, self.ptgt_h,self.sensing_h, self.I_star_h,
#
#     def setupAnimation(self, fig, ax, bounds):
#         self.fig = fig
#         self.ax = ax
#         self.bounds = bounds
#         self.p1_h = ax.scatter(0,0, color = 'blue')
#         self.p1_traj_h, = ax.plot([], [], 'b-')
#         self.pA_h = ax.scatter(0,0, color = 'red')
#         self.pA_traj_h, = ax.plot([], [], 'r-')
#         self.ptgt_h = ax.scatter(0,0, color = 'black')
#         self.sensing_h, = ax.plot([], [], 'k-')
#         self.I_star_h, = ax.plot([], [], 'b*')
#         #return self.p1_h,
