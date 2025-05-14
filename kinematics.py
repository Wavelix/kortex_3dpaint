import numpy as np

PI=np.pi

class Kinematics:
    def __init__(self):
        self.alpha=np.array([0,0.5*PI,PI,-0.5*PI,-0.5*PI,-0.5*PI])
        self.a=np.array([0,0,280,0,0,0])
        self.d=np.array([243.3,30,20,245,28.5*2,235])
        # self.alpha=np.array([0,0.5*PI,PI,0.5*PI,0.5*PI,-0.5*PI])
        # self.a=np.array([0,0,280,0,0,0])
        # self.d=np.array([243.3,10,0,245,28.5*2,235])

        self.thetas=np.array([[0,345,75,0,300,0],
                [0,0,0,0,0,0],
                [357,21,150,272,320,273],
                [270,148,148,270,140,0],
                [20.5,313.5,100,265.5,327,57]])
        self.names=np.array(["HOME","ZERO","RETRACT","PACKAGING","PICK"])
        
    def transform(self,theta):
        theta=theta/180*PI
        theta+=np.array([0,PI/2,-PI/2,PI/2,PI,0])
        # theta+=np.array([0,PI/2,PI/2,PI/2,0,-PI/2])
        T=np.eye(4)
        for i in range(6):
            t=np.array([[np.cos(theta[i]),-np.sin(theta[i]),0,self.a[i]],
                        [np.sin(theta[i])*np.cos(self.alpha[i]),np.cos(theta[i])*np.cos(self.alpha[i]),-np.sin(self.alpha[i]),-np.sin(self.alpha[i])*self.d[i]],
                        [np.sin(theta[i])*np.sin(self.alpha[i]),np.cos(theta[i])*np.sin(self.alpha[i]),np.cos(self.alpha[i]),np.cos(self.alpha[i])*self.d[i]],
                        [0,0,0,1]])
            T=T@t
        np.set_printoptions(precision=3, suppress=True)
        print(T) 

    def main(self):
        for i in range(len(self.names)):
            theta=self.thetas[i]
            print(self.names[i]+":")
            self.transform(theta)
            print("\n")

if __name__=="__main__":
    kinematics=Kinematics()
    kinematics.main()