import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.pyplot as plot

class LinearQuadraticRegulator:
    def __init__(self):
        self.v=6
        self.L=2.85
        self.T=0.05
        self.N=200
        self.Nx=3
        self.Nu=1
        self.r=20
        self.Np=10
        self.CEN=[0,0]
        self.start_i=0
        self.FWA=np.zeros((self.N+1,1))
        self.Xout=np.zeros((self.N,self.Nx))
        self.x_real=np.zeros((self.N+1,self.Nx))

        self.x_real[0,:]=np.array([10,10,math.pi/2])
        self.Xout = self.getCircularRefTraj(self.x_real[0,0],self.x_real[0,1],self.N)

        self.Delta_x=np.zeros((3,1))
        self.Q=np.array([[10,0,0],[0,10,0],[0,0,100]])
        self.R=np.array([10])
        self.Pk=np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.Vk=np.array([0,0,0])

    def Func_Alpha_Pos(self,Xb,Yb,Xn,Yn):
        AngleY = Yn - Yb
        AngleX = Xn - Xb
        if Xb == Xn:
            if Yn > Yb:
                K = math.pi / 2
            else:
                K = 3 * math.pi / 2
        else:
            if Yb == Yn:
                if Xn > Xb:
                    K = 0
                else:
                    K = math.pi
            else:
                K = math.atan(AngleY / AngleX)
        if AngleY > 0 and AngleX > 0:
            K = K
        elif (AngleY > 0 and AngleX < 0) or (AngleY < 0 and AngleX < 0):
            K = K + math.pi
        elif (AngleY < 0 and AngleX > 0):
            K = K + 2 * math.pi
        else:
            K = K
        return K

    def Func_Theta_Pos(self,Alpha):
        if Alpha >= 3*math.pi/2:
            Theta = Alpha-3*math.pi/2
        else:
            Theta = Alpha+math.pi/2
        return Theta

    def getCircularRefTraj(self,Pos_x,Pos_y,N):
        RefTraj=np.zeros((N,4))
        Alpha_init=self.Func_Alpha_Pos(self.CEN[0],self.CEN[1],Pos_x,Pos_y)
        Omega=self.v/self.r
        DFWA=math.atan(self.L/self.r)
        for k in range(N):
            Alpha=Alpha_init+Omega*self.T*(k-1)
            RefTraj[k,0]=self.r*math.cos(Alpha)+self.CEN[0]
            RefTraj[k,1]=self.r*math.sin(Alpha)+self.CEN[1]
            RefTraj[k,2]=self.Func_Theta_Pos(Alpha)
            RefTraj[k,3]=DFWA
        return RefTraj

    def findStart(self,x,y):
        distance=((x-self.Xout[0,0])**2+(y-self.Xout[0,1])**2)**0.5
        for i in range(self.start_i,self.N):
            dis=((x-self.Xout[i,0])**2+(y-self.Xout[i,1])**2)**0.5
            if dis<distance:
                distance=dis
                self.start_i=i

    def getXYZ(self,x, y, heading, FWA, DFWA):
        num = 100
        Xmc = np.zeros((1, num))
        Ymc = np.zeros((1, num))
        Headingmc = np.zeros((1, num))
        Xmc[0] = x
        Ymc[0] = y
        Headingmc[0]= heading
        Headingrate = np.zeros((1, num))
        FrontWheelAngle = np.zeros((1, num))
        t = self.T / num
        FrontWheelAngle = np.linspace(FWA, DFWA, num)
        Headingrate = self.v * np.tan(FrontWheelAngle) / self.L
        for i in range(1,num):
            Headingmc[0,i] = Headingmc[0,i - 1] + Headingrate[i] * t
            Xmc[0,i] = Xmc[0,i - 1] + self.v * t * math.cos(Headingmc[0,i - 1])
            Ymc[0,i] = Ymc[0,i - 1] + self.v * t * math.sin(Headingmc[0,i - 1])
        X = Xmc[0,-1]
        Y = Ymc[0,-1]
        H = Headingmc[0,-1]
        return np.array([X,Y,H])

    def matToNumber(self,mat1,mat2):
        mat=np.array(np.dot(np.mat(mat1),np.mat(mat2).T))
        return np.array(np.dot(np.mat(mat1),np.mat(mat2).T))[0,0]

    def matMulti(self,mat1,mat2,mat3=None,parameter=None):
        mat11=np.mat(mat1)
        mat22=np.mat(mat2)
        mat=None
        if mat3 is not None:
            mat33=np.mat(mat3)
            if parameter is None:
                mat=np.dot(np.dot(mat11,mat22),mat33)
            elif parameter == 'PTAP':
                mat=np.dot(np.dot(mat11.T,mat22),mat33)
            elif parameter == 'PAPT':
                mat=np.dot(np.dot(mat11,mat22),mat33.T)
        else:
            if parameter is None:
                mat=np.dot(mat11,mat22)
            elif parameter=='ATA':
                mat=np.dot(mat11.T,mat22)
            elif parameter=='AAT':
                mat=np.dot(mat11,mat22.T)
            elif parameter=='ATAT':
                mat = np.dot(mat11.T, mat22.T)
        if mat.shape == (1,1):
            return np.array(mat)[0][0]
        else:
            return np.array(mat)

    def LQR(self):
        DFWA = math.atan(self.v / self.L)
        forecastN=3
        K=None
        Ku=None
        Kv=None
        Vk_1=None
        Delta_x=np.zeros((1,self.Nx))
        self.findStart(self.x_real[0, 0], self.x_real[0, 1])
        for i in range(self.N):
            forecastTraj=self.getCircularRefTraj(self.x_real[i,0],self.x_real[i,1],self.Np)
            Delta_x[0,0]=self.x_real[i,0]-forecastTraj[forecastN,0]
            Delta_x[0,1]=self.x_real[i,1]-forecastTraj[forecastN,1]
            Delta_x[0,2]=self.x_real[i,2]-forecastTraj[forecastN,2]
            u_feedForward=forecastTraj[forecastN,3]
            if Delta_x[0,2]>math.pi:
                Delta_x[0,2]=Delta_x[0,2]-2*math.pi
            elif Delta_x[0,2]<-1*math.pi:
                Delta_x[0,2]=Delta_x[0,2]+2*math.pi
            for j in range(self.Np,1,-1):
                Pk_1=self.Pk
                Vk_1=self.Vk
                A=np.array([[1,0,-self.v*self.T*math.sin(forecastTraj[j-1,2])],
                            [0,1,self.v*self.T*math.cos(forecastTraj[j-1,2])],
                            [0,0,1]])
                B=np.array([0,0,self.v*self.T/(self.L*(math.cos(forecastTraj[j-1,2]))**2)])
                BKB=1/(self.matMulti(B,Pk_1,B,'PAPT')+self.R)
                K=BKB*self.matMulti(B,Pk_1,A)
                Ku=self.matMulti(BKB,self.R)
                Kv=BKB*B.T
                ATP=self.matMulti(A,Pk_1,parameter='ATA')
                A_BK=A-self.matMulti(B,K,parameter='ATA')
                self.Pk=self.matMulti(ATP,A_BK)+self.Q
                self.Vk = np.ravel(self.matMulti(A_BK, Vk_1, parameter='ATAT')) - K * self.R * forecastTraj[j - 1, 3]
            u_feedBackward = -self.matMulti(K, Delta_x, parameter=('AAT')) - Ku * u_feedForward - self.matMulti(Kv,Vk_1,parameter=('AAT'))
            self.FWA[i+1,:]=u_feedBackward+u_feedForward
            self.x_real[i+1,:]=self.getXYZ(self.x_real[i,0],self.x_real[i,1],self.x_real[i,2],self.FWA[i,0],self.FWA[i+1,0])

if __name__ == '__main__':
    LQR=LinearQuadraticRegulator()
    LQR.LQR()
    plt.figure(3)
    plt.plot(LQR.x_real[:,0],LQR.x_real[:,1],"*")
    plt.plot(LQR.Xout[:,0],LQR.Xout[:,1])
    plt.savefig('LinearQuadraticRegulator.png',dpi=700)
    plt.show()