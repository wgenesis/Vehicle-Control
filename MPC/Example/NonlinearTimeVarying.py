import numpy as np
import math
from scipy import optimize as op
import matplotlib.pyplot as plt

class NonlinearTimeVaryingMPC:
    def __init__(self):
        self.Nx=3       #状态量个数
        self.Np=15      #预测时域
        self.Nc=2       #控制量个数
        self.L=1        #轴距
        self.Q=10*np.eye(self.Np+1)
        self.R=10*np.eye(self.Np+1)
        self.N=100      #仿真次数
        self.T=0.05     #时间间隔
        self.x_real = np.zeros((self.N+1, self.Nx))   #真实位置
        self.x_real[0,:]=[0,0,math.pi/6]            #初始位置
        self.Xout=np.zeros((self.N,self.Nx))        #轨迹点序列
        for i in range(self.N):
            self.Xout[i,0]=i*self.T
            self.Xout[i,1]=2
            self.Xout[i,2]=0

        self.lb = np.array([0.7, -0.44])
        self.ub = np.array([5, 0.8])
        self.initX=np.array([2,0,2,0])
        self.position=[1,25,5]


        self.State_Initial=np.zeros((self.N,3))
        self.State_Initial[0,:]=[0,0,math.pi/6]
        self.Xref=np.zeros((self.Np,1))
        self.Yref=np.zeros((self.Np,1))
        self.PHIref=np.zeros((self.Np,1))

    def getCostFunction(self, x, R, Q,n):
        cost = 0
        [X, Y, PHI] = self.x_real[n, :]
        X_predict = np.zeros((self.Np, 1))
        Y_predict = np.zeros((self.Np, 1))
        PHI_predict = np.zeros((self.Np, 1))
        X_error = np.zeros((self.Np + 1, 1))
        Y_error = np.zeros((self.Np + 1, 1))
        PHI_error = np.zeros((self.Np + 1, 1))

        v = np.zeros((self.Np, 1))
        delta_f = np.zeros((self.Np, 1))

        for i in range(self.Np):
            if i == 0:
                v[i, 0] = x[0]
                delta_f[i, 0] = x[1]
                X_predict[i, 0] = X + self.T * v[i, 0] * math.cos(PHI)
                Y_predict[i, 0] = Y + self.T * v[i, 0] * math.sin(PHI)
                PHI_predict[i, 0] = PHI + self.T * v[i, 0] * math.tan(delta_f[i, 0]) / self.L
            else:
                v[i, 0] = x[2]
                delta_f[i, 0] = x[3]
                X_predict[i, 0] = X_predict[i - 1] + self.T * v[i, 0] * math.cos(PHI_predict[i - 1])
                Y_predict[i, 0] = Y_predict[i - 1] + self.T * v[i, 0] * math.sin(PHI_predict[i - 1])
                PHI_predict[i, 0] = PHI_predict[i - 1] + self.T * v[i, 0] * math.tan(delta_f[i, 0]) / self.L

            X_real = np.zeros((self.Np + 1, 1))
            Y_real = np.zeros((self.Np + 1, 1))
            X_real[0, 0] = X
            X_real[1:self.Np+1, 0] = np.ravel(X_predict)
            Y_real[0, 0] = Y
            Y_real[1:self.Np+1, 0] = np.ravel(Y_predict)
            X_error[i, 0] = X_real[i, 0] - self.Xout[i, 0]
            Y_error[i, 0] = Y_real[i, 0] - self.Xout[i, 1]
            PHI_error[i, 0] = PHI_predict[i, 0] - self.Xout[i, 2]

            cost = cost + np.dot(np.dot(Y_error.T, R) , Y_error)*self.position[1] + np.dot(np.dot(X_error.T, Q), X_error)*self.position[0]+np.dot(np.dot(PHI_error.T,Q),PHI_error)*self.position[2]
        return cost[0][0]

    def getCon(self):
        cons=({'type':'ineq','fun':lambda x:x[0]-self.lb[0]},
             {'type':'ineq','fun':lambda x:-x[0]+self.ub[0]},
             {'type':'ineq','fun':lambda x:x[2]-self.lb[0]},
             {'type':'ineq','fun':lambda x:-x[2]+self.ub[0]},
             {'type':'ineq','fun':lambda x:x[1]-self.lb[1]},
             {'type':'ineq','fun':lambda x:-x[1]+self.ub[1]},
             {'type':'ineq','fun':lambda x:x[3]-self.lb[1]},
             {'type':'ineq','fun':lambda x:-x[3]+self.ub[1]})
        return cons

    def getXYZ(self,X00,v,deltaf):
        if deltaf==0:
            deltaf=0.00001
        X=X00[0] - math.sin(X00[2])/math.tan(deltaf) + math.sin(X00[2] + self.T*v*math.tan(deltaf))/math.tan(deltaf)
        Y=X00[1] + math.cos(X00[2])/math.tan(deltaf) - math.cos(X00[2] + self.T*v*math.tan(deltaf))/math.tan(deltaf)
        Z=X00[2] + self.T*v*math.tan(deltaf)
        return np.array([X,Y,Z])

    def MPC(self):
        X00=np.zeros((self.Nx,1))
        for j in range(self.N):
            cons=self.getCon()
            res=op.minimize(fun=self.getCostFunction,x0=self.initX,args=(self.R,self.Q,j),constraints=cons)
            v=res.x[0]
            deltaf=res.x[1]
            X00[0,0]=self.x_real[j,0]
            X00[1,0]=self.x_real[j,1]
            X00[2,0]=self.x_real[j,2]
            self.x_real[j+1,:]=np.ravel(self.getXYZ(X00,v,deltaf))

if __name__ == '__main__':
    nonLinearMPC=NonlinearTimeVaryingMPC()
    nonLinearMPC.MPC()
    plt.figure(2)
    plt.plot(nonLinearMPC.x_real[:, 0], nonLinearMPC.x_real[:, 1], "*")
    plt.plot(nonLinearMPC.Xout[:, 0], nonLinearMPC.Xout[:, 1])
    plt.savefig("NonlinearTimeVaryingMPC.png", dpi=700)
    plt.show()