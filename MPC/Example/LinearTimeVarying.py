from matplotlib import pyplot as plt
import numpy as np
import math
import cvxopt as cvx
from scipy import optimize as op

class LinearTimeVaryingMPC:
    def __init__(self):
        self.N=100
        self.T=0.05
        self.Xout=np.zeros((self.N,3))      #轨迹点序列
        self.Tout=np.zeros((self.N,1))      #时间序列
        for k in range(self.N):
            self.Xout[k,0]=(k+1)*self.T     #目标轨迹的X坐标
            #self.Xout[k,1]=0.5*self.Xout[k,0]**2                #目标轨迹的Y坐标
            self.Xout[k,1]=(25-(self.Xout[k,0]-5)**2)**0.5
            #self.Xout[k,1]=2
            if k>0:
                self.Xout[k-1,2]=math.atan((self.Xout[k,1]-self.Xout[k-1,1])/(self.Xout[k,0]-self.Xout[k-1,0]))               #目标轨迹的航向角
            # else:
            #     self.Xout[k,2]=math.atan((self.Xout[k,1]-self.Xout[k-1,1])/(self.Xout[k,0]-self.Xout[k-1,0]))
            self.Tout[k,0]=(k-1)*self.T     #轨迹点对应时刻，100*0.05=5s
        self.Nx=3                           #状态量个数
        self.Nu=2                           #控制量个数
        [self.Nr,self.Nc]=self.Xout.shape   #目标点的数目：Nr=100  状态量的数目：Nc=3
        self.Tsim=20                        #预测时域
        self.X0=[0,0,math.pi/3]             #车辆初始状态，坐标（0，0），航向角pi/3
        self.L=1                            #车辆轴距
        self.vd1=1                          #纵向速度
        self.vd2=0                          #前轮偏角
        self.x_real=np.zeros((self.Nr,self.Nc))                   #每一个仿真时刻车辆的位置状态 100*3
        self.x_real[0, :] = self.X0                               #初始时刻，车辆为起始点位置
        self.x_piao=np.zeros((self.Nr,self.Nc))                   #每一个仿真时刻，车辆位置与目标位置的误差 100*3
        self.x_piao[0, :] = self.x_real[0, :] - self.Xout[0, :]   #初始时刻的初始偏差
        self.X_PIAO = np.zeros((self.Nr, self.Nx*self.Tsim))      #每一个仿真时刻用于预测的偏差 100*60
        self.u_real=np.zeros((self.Nr,2))                   #每一个仿真时刻车辆的真实控制量 100*3
        self.u_piao=np.zeros((self.Nr,2))                   #每一个仿真时刻车辆的真实控制量与目标控制量的偏差 100*3

        self.XXX=np.zeros((self.Nr,self.Nx*self.Tsim))            #每一时刻所有预测的状态 100*60
        self.q=np.array([[1,0,0],[0,1,0],[0,0,0.5]])              #加权矩阵，控制X，Y，航向角的权重 3*3
        self.Q=np.zeros((self.Tsim*self.Nx,self.Tsim*self.Nx))    #加权矩阵，60*60
        for i in range(self.Tsim*self.Nx):
            for j in range(self.Tsim*self.Nx):
                if i==j and i%self.Nx==0:
                    self.Q[i:i+3,j:j+3]=self.q
        self.R=0.1*np.eye(self.Nu*self.Tsim)       #最终的状态加权矩阵，用于调整控制量偏差和状态量偏差的比重

    def matrixPower(self,mat,N):
        if mat.shape[0]!=mat.shape[1]:
            raise Exception("Inconsistent dimension of matrix!")
        else:
            mat_out=np.eye(mat.shape[0])
            for i in range(N):
                mat_out=np.dot(mat_out,mat)
            return mat_out

    def quadprog(self,H,f,A_cons,b_cons,a1,a2,lb,ub):
        n=H.shape[1]
        P=H
        q=f
        G=np.vstack([-np.eye(n),np.eye(n)])
        h=np.array(-lb)
        for i in range(self.Tsim-1):
            h=np.hstack([h,-lb])
        for i in range(self.Tsim):
            h=np.hstack([h,ub])
        A=a1
        b=a2
        sol=cvx.solvers.qp(cvx.matrix(P),cvx.matrix(q),cvx.matrix(G),cvx.matrix(h))
        x=sol['x']
        return x

    def getXYZ(self,X00,vd11,vd22,t):
        x=X00[0] + (vd11 * math.sin(X00[2] + t * vd22)) / vd22 - (vd11 * math.sin(X00[2])) / vd22
        y=X00[1] - (vd11*math.cos(X00[2] + t*vd22))/vd22 + (vd11*math.cos(X00[2]))/vd22
        z=X00[2] + t*vd22
        return x,y,z

    def MPC(self):
        for i in range(self.Nr):
            t_d=self.Xout[i,2]  #目标轨迹航向角
            #a,b 离散化的运动学模型
            #下一预测点的状态偏差 = a*当前点的状态偏差 + b*当前点的控制量偏差   （状态偏差，即车辆位置偏差；控制量偏差，即车辆运动偏差）
            a=np.array([[1,0,-self.T*self.vd1*math.sin(t_d)],
                       [0,1,self.T*self.vd1*math.cos(t_d)**2],
                       [0,0,1]])
            b=np.array([[math.cos(t_d)*self.T,0],
                        [math.sin(t_d)*self.T,0],
                        [math.tan(self.vd2)*self.T/self.L,self.vd1*self.T/(self.L*math.cos(self.vd2)**2)]])
            # b = np.array([[math.cos(self.Xout[i, 2]) * self.T, 0],
            #              [math.sin(self.Xout[i,2])*self.T,0],
            #              [0,self.T]])
            #目标函数，是预测时域内预测点方差之和；
            #预测时域为Tsim，即存在20个预测点；
            #预测点方差为“状态方差（位置方差）” + “控制量方差（运动方差）”；
            A=np.zeros([self.Tsim*self.Nx,self.Nx])
            B=np.zeros([self.Tsim*self.Nx,self.Tsim*self.Nu])
            for j in range(self.Tsim*self.Nx):
                if j==0:
                    A[0:3,:]=a
                elif j%self.Nx==0:
                    A[j:j+3,:]=np.dot(A[j-3:j,:],a)
                for k in range(self.Tsim):
                    if k<= j/3 and j%3==0:
                        jj=int(j/3)
                        if 0 not in B[jj*3:jj*3+3,k*2:k*2+2].shape:
                            B[jj*3:jj*3+3,k*2:k*2+2]=np.dot(self.matrixPower(a,jj-k),b)
                       # B[j,k]=(a**(j-k))*b
            H=2*(np.dot(np.dot(B.T,self.Q),B)+self.R)
            f=2*np.dot(np.dot(np.dot(B.T,self.Q),A),self.x_piao[i,:].T)
            A_cons=[]
            b_cons=[]
            lb=np.array([-1,-1])
            ub=np.array([1.1,1])
            X=self.quadprog(H,f,A_cons,b_cons,[],[],lb,ub)
            X=np.array(X)
            print(X[0],X[1])
            bx=np.dot(B,X)
            bx=bx.ravel()
            self.X_PIAO[i,:]=np.dot(A,self.x_piao[i,:])+bx
            if i+self.Tsim<self.Nr:
                for j in range(self.Tsim):
                    self.XXX[i,3*j]=self.X_PIAO[i,3*j]+self.Xout[i+j,0]
                    self.XXX[i,3*j+1]=self.X_PIAO[i,3*j+1]+self.Xout[i+j,1]
                    self.XXX[i,3*j+2]=self.X_PIAO[i,3*j+2]+self.Xout[i+j,2]
            else:
                for j in range(self.Tsim):
                    self.XXX[i,3*j]=self.X_PIAO[i,3*j]+self.Xout[self.Nr-1,0]
                    self.XXX[i,3*j+1]=self.X_PIAO[i,3*j+1]+self.Xout[self.Nr-1,1]
                    self.XXX[i,3*j+2]=self.X_PIAO[i,3*j+2]+self.Xout[self.Nr-1,2]

            self.u_piao[i,0]=X[0]
            self.u_piao[i,1]=X[1]
            Tvec=np.arange(0,4,0.05)
            X00=self.x_real[i,:]
            vd11=self.vd1+self.u_piao[i,0]
            vd22=self.vd2+self.u_piao[i,1]

            if i<self.Nr-1:
                self.x_real[i + 1, 0], self.x_real[i + 1, 1], self.x_real[i + 1, 2] = self.getXYZ(X00, vd11, vd22, self.T)
                self.x_piao[i+1,:]=self.x_real[i+1,:]-self.Xout[i+1,:]
            self.u_real[i,0]=self.vd1+self.u_piao[i,0]
            self.u_real[i,1]=self.vd2+self.u_piao[i,1]

if __name__ == '__main__':
    linearMPC=LinearTimeVaryingMPC()
    linearMPC.MPC()
    plt.figure(1)
    plt.plot(linearMPC.x_real[:, 0], linearMPC.x_real[:, 1], "*")
    plt.plot(linearMPC.Xout[:, 0], linearMPC.Xout[:, 1])
    plt.savefig("LinearTimeVaryingMPC.png",dpi=700)
    plt.show()