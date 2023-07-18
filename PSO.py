import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import  random
class Train_Road():
    def __init__(self,_TrainMass,_Trainlength,_Geartable,_Roadlength,_PlanningTime,_Roadlimited,_StationLimited,_SlopeTable,_Stationlength):
        self.Roadlength = _Roadlength #路长
        self.Trainmass = _TrainMass
        self.Trainlength =_Trainlength
        self.Geartable = _Geartable
        self.Roadlimited = _Roadlimited#道路限速
        self.Stationlimited = _StationLimited#车站限速
        self.Slopetable = _SlopeTable#坡度表
        self.Planningtime = _PlanningTime
        self.Stationlength =_Stationlength
        self.Road = np.arange(0,int(self.Roadlength)+1,1)
        self.Traction = np.zeros(math.floor(self.Roadlength) + 1, dtype=float)#列车牵引力，就是列车自身的加速度
        self.Accle = np.zeros(math.floor(self.Roadlength) + 1, dtype=float)#加速度
        self.TrainSpeed = np.zeros(math.floor(self.Roadlength) + 1, dtype=float)#速度
        self.RunningTime = np.zeros(math.floor(self.Roadlength) + 1, dtype=float)#行驶时间
        self.Energy = np.zeros(math.floor(self.Roadlength) + 1, dtype=float)#能耗
        self.RoadAccele = []
        self.RoadSpeedLimited = []
        self.SlopChangepoints = []
        self.Totaltime = 0
        self.Totaldistance = 0
        self.Totalenergy = 0

        for i in range(math.floor(self.Roadlength) + 1):
            if i <= self.Stationlength or i >= (math.floor(self.Roadlength) - self.Stationlength + 1):
                self.RoadSpeedLimited.append(self.Stationlimited)
            else:
                self.RoadSpeedLimited.append(self.Roadlimited)
        self.limitedChangepoints = []
        for i in range(math.floor(self.Roadlength) + 1):
            if self.RoadSpeedLimited[i] < self.RoadSpeedLimited[i - 1]:
                self.limitedChangepoints.append(i)
            accspeed = self.Slopetable[i]*Gravity/1000
            self.RoadAccele.append(accspeed)
        self.RoadAccele = np.array(self.RoadAccele)
        # 第二种计算方式，考虑车站长度
        # for i in range(len(self.Slopetable)):
        #     if i > 0 and self.Slopetable[i] != self.Slopetable[i - 1]:
        #         self.SlopChangepoints.append(i)
        # for i in range(math.floor(self.Roadlength) + 1):
        #     accSpeed = 0
        #     if i <= self.Trainlength:
        #         for j in range(0, i):
        #             accSpeed += self.Slopetable[j] * Gravity / (self.Trainlength * 1000.)
        #         self.RoadAccele.append(accSpeed)
        #     else:
        #         for j in range(i - 120, i):
        #             accSpeed += self.Slopetable[j] * Gravity / (self.Trainlength * 1000.)
        #         self.RoadAccele.append(accSpeed)
        # self.RoadAccele = np.array(self.RoadAccele)
        for i in self.limitedChangepoints:
            tempv1 = self.RoadSpeedLimited[i] / 3.6
            tempTraction = -0.8
            tempv2 = tempv1 ** 2 - 2 * (tempTraction - self.RoadAccele[i] - (
                        2.7551 + 0.014 * tempv1 + 0.00075 * tempv1 * tempv1)  / (310*1000))
            tempv2 = math.sqrt(tempv2)
            while (tempv2 * 3.6 <= self.RoadSpeedLimited[i - 1]):
                tempv1 = tempv2
                self.RoadSpeedLimited[i] = tempv2 * 3.6
                tempv2 = tempv1 ** 2 - 2 * (tempTraction - self.RoadAccele[i] - (
                        2.965 -0.064 * tempv1 + 3.858 * tempv1 * tempv1)  /(310*1000))
                tempv2 = math.sqrt(tempv2)
                i = i - 1
        self.RoadSpeedLimited = np.array(self.RoadSpeedLimited)

    def run(self, Road_Train_Gear):
        overspeedtimes = 0
        backtag =0
        self.runningdistance = 0.
        for i in range(math.floor(self.Roadlength) + 1):
            self.Traction[i] = Road_Train_Gear[i]
        for i in range(self.Road.shape[0]):
            distance = 0.
            if i == 0:
                self.Accle[i] = self.Traction[i] - self.RoadAccele[i]
                self.TrainSpeed[i] = 0.
                self.RunningTime[i] = 0.
                self.Energy[i] = 0.
            elif i == 1:
                self.Accle[i] = self.Traction[i] - self.RoadAccele[i]
                self.TrainSpeed[i] = math.sqrt(abs(self.Accle[i] * 2))
                self.RunningTime[i] = self.TrainSpeed[i] / self.Accle[i]
                distance = self.TrainSpeed[i] ** 2 / (2 * self.Accle[i])
                if self.Traction[i] < 0:
                    self.Energy[i] = self.Trainmass * 1000. * self.Traction[i] * 0.15
                else:
                    self.Energy[i] = self.Trainmass * 1000. * self.Traction[i] / 0.75
            else:
                vm = self.TrainSpeed[i - 1] * 3.6
                self.Accle[i] = self.Traction[i] - self.RoadAccele[i] - (
                        2.965-0.064* vm + 3.858 * vm * vm) /(1000.*310)
                if vm >= 0 and (2 * self.Accle[i - 1] + self.TrainSpeed[i - 1] ** 2) > 0:
                    self.TrainSpeed[i] = math.sqrt(2 * self.Accle[i - 1] + self.TrainSpeed[i - 1] ** 2)
                    self.RunningTime[i] = abs((self.TrainSpeed[i] - self.TrainSpeed[i - 1]) / self.Accle[i - 1])
                    distance = abs(
                        (self.TrainSpeed[i] ** 2 - self.TrainSpeed[i - 1] ** 2) / (2 * self.Accle[i - 1]))
                    if self.Traction[i] < 0:
                        self.Energy[i] = self.Trainmass * 1000. * self.Traction[i] * 0.15 * distance
                    else:
                        self.Energy[i] = self.Trainmass * 1000. * self.Traction[i] / 0.75 * distance
                if (2 * self.Accle[i - 1] + self.TrainSpeed[i - 1] ** 2) <= 0:
                    self.TrainSpeed[i] = 0
                    self.RunningTime[i] = abs((self.TrainSpeed[i] - self.TrainSpeed[i - 1]) / self.Accle[i - 1])
                    distance = abs(
                        (self.TrainSpeed[i] ** 2 - self.TrainSpeed[i - 1] ** 2) / (2 * self.Accle[i - 1]))
                    if self.Traction[i] < 0:
                        self.Energy[i] = self.Trainmass * 1000. * self.Traction[i] * 0.15 * distance
                    else:
                        self.Energy[i] = self.Trainmass * 1000. * self.Traction[i] / 0.75 * distance
                    backtag+=1
            self.runningdistance += distance
            if self.TrainSpeed[i]*3.6>self.RoadSpeedLimited[i]:
                overspeedtimes+=1

            if i == self.Road.shape[0] - 1:
                vm = self.TrainSpeed[i]
                lastAccle = self.Accle[i]
                lastrunningtime = abs(vm / lastAccle)
                lastdistance = abs(vm ** 2 / (2 * lastAccle))
                lastEnergy = -self.Trainmass * 1000. * self.Traction[i] * 0.15 * lastdistance
                if backtag>=5:
                    self.runningdistance = 0
                else:
                    self.runningdistance+=lastdistance

                self.Totalenergy = np.sum(self.Energy) + lastEnergy
                self.Totaltime = np.sum(self.RunningTime) + lastrunningtime
        return self.Totalenergy,self.Totaltime,self.runningdistance,overspeedtimes

class PSO(object):
    def __init__(self, population_size, max_steps,T_R):
        self.w = 0.7 # 惯性权重
        self.c1 = self.c2 = 2
        self.population_size = population_size  # 粒子群数量
        self.dim1 = 8  # 位置粒子空间的维度
        self.dim2 = 9   #挡位粒子
        self.max_steps = max_steps  # 迭代次数
        self.x=[]
        for i in range(200):
            while True:
                temp = 1
                x1 = np.round(np.random.rand(self.dim1) * 1982)#位置粒子
                x1 = np.sort(x1)
                for j in range(7):#检验粒子是否存在第一个为0 或者前后两者相同
                    if j ==0 and x1[j]==0:
                        temp=0
                    if x1[j] == x1[j + 1]:
                        temp = 0
                if temp == 1:
                    self.x.append(x1)
                    break # 初始化粒子群位置
        self.x = np.array(self.x)
        # self.x2_bound = [0,11]#挡位粒子
        self.x2 = np.random.random((self.population_size,self.dim2)) * 2.0 - 1.0
        # self.x2 = self.check1(self.x2,T_R)#粒子一定要符合列车运行规则
        self.v = np.random.rand(self.population_size, self.dim1)# 初始化位置粒子群速度
        self.v2 = np.random.rand(self.population_size,self.dim2)#初始化挡位粒子群速度
        self.kexj = []

        fitness = self.calculate_fitness(self.x,self.x2,T_R)
        self.p = self.x  # 个体的最佳位置
        self.p2 =self.x2
        self.pg = self.x[np.argmin(fitness)]  # 全局最佳位置
        self.pg2 = self.x2[np.argmin(fitness)]
        self.individual_best_fitness = fitness  # 个体的最优适应度
        self.global_best_fitness = np.min(fitness) # 全局最佳适应度


    def calculate_fitness(self,x1,x2,Train_road):
        # x1*=50
        gear = np.zeros((self.population_size, 1982 + 1))
        fitness=np.zeros(self.population_size)
        for i in range(self.population_size):
            for j in range(self.dim1):
                if j ==0:
                    for ij in range(int(x1[i,j])):
                        gear[i,ij]=x2[i,j]
                elif j<self.dim1:
                    for ij in range(int(x1[i,j-1]),int(x1[i,j])):
                        gear[i,ij]=x2[i,j]
                    if j==self.dim1-1:
                        for ij in range(int(x1[i,j]),gear.shape[1]):
                            gear[i,ij]=x2[i,j+1]
        for i in range(self.population_size):
            E,T,D,O=Train_road.run(gear[i])#计算各个指标，能耗，时间，距离，超速次数
            if abs(D-Train_road.Roadlength)<1 and abs(T-Train_road.Planningtime)<5 and O<=1 :
                zhibiao = np.reshape([E,T,D],(1,-1))
                points =np.reshape(x1[i],(1,-1))
                gears =np.reshape(x2[i],(1,-1))
                self.kexj.append((np.c_[points,gears,zhibiao]))
                # plt.ion()
                # X1 = TR.Road
                # X2= np.arange(0, int(TR.Roadlength)+2, 1)
                # X2[-1] = TR.runningdistance
                # y2 = TR.RoadSpeedLimited
                # y = TR.TrainSpeed * 3.6
                # y = np.append(y, 0)
                # plt.plot(X2, y)
                # plt.plot(X1, y2)
                # plt.pause(2)
                # plt.close()

                print(np.c_[points,gears,zhibiao])
            fitness[i] = 10*(max(0,abs(D-Train_road.Roadlength)-1))+(max(0,abs(T-Train_road.Planningtime)-5))+max(0,O)
        # x1/=50

        return fitness

    def check1(self,x,T_R):
        for i in range(x.shape[0]):
            temp = 1
            while temp:
                if x[i,0]>=0 and x[i,0]<=4 and x[i,-1]<=10 and x[i,-1]>=6:
                    temp2 =0
                    for j in range(x.shape[1]-1):
                        if (T_R.Geartable[x[i,j]] * T_R.Geartable[x[i,j+1]])<0 :
                            temp2 =1
                            break
                    if temp2==0:
                        break

                x[i] = np.random.randint(0,11,9)
        return x
    def check2(self,x,T_R):
        for i in range(x.shape[0]-1):
            if x[i]*x[i+1]<0:
                if abs(x[i])>=abs(x[i+1]):
                    x[i]=0.0
                else:
                    x[i+1]=0.0
        return x

    def evolve(self):
        for step in range(self.max_steps):
            r1 = np.random.rand(self.population_size, self.dim1)
            r2 = np.random.rand(self.population_size, self.dim1)
            # 更新速度和权重
            self.v =np.round(self.w * self.v + self.c1 * r1 * (self.p - self.x) + self.c2 * r2 * (self.pg - self.x))
            for i in range(self.v.shape[0]):
                for j in range(self.v.shape[1]):
                    if self.v[i,j]>200:
                        self.v[i,j]=200
                    elif self.v[i,j]<-200:
                        self.v[i,j]=-200
            self.x = self.v + self.x
            self.x = np.sort(self.x)
            for i in range(self.x.shape[0]):
                for j in range(self.x.shape[1]):
                    if self.x[i,j]>1982:
                        self.x[i,j]=1982
                    elif self.x[i,j]<1:
                        self.x[i,j]=1

            r3 = np.random.rand(self.population_size, self.dim2)
            r4 = np.random.rand(self.population_size, self.dim2)
            self.v2 = self.w * self.v2 + self.c1 * r3 * (self.p2 - self.x2) + self.c2 * r4 * (self.pg2 - self.x2)
            for i in range(self.v2.shape[0]):
                for j in range(self.v2.shape[1]):
                    if self.v2[i,j]<-0.5:
                        self.v2[i,j] = max(-1,self.v2[i,j])
                    elif self.v2[i,j]>0.5:
                        self.v2[i,j] = min(1, self.v2[i,j])
                    # else: self.v2[i,j] =0

            self.x2 = self.v2 + self.x2

            for i in range(self.x2.shape[0]):
                for j in range(self.x2.shape[1]):
                    if self.x2[i,j]<-0.5:
                        self.x2[i,j]=max(-1, self.x2[i,j])
                    elif self.x2[i,j]>0.5:
                        self.x2[i,j]=min(1, self.x2[i,j])
            for i in range(self.x2.shape[0]):
                self.x2[i]=self.check2(self.x2[i],TR)
            # plt.clf()
            # # plt.scatter(self.x[:, 0], self.x[:, 1], s=30, color='k')
            # # plt.xlim(self.x_bound[0], self.x_bound[1])
            # # plt.ylim(self.x_bound[0], self.x_bound[1])
            # # plt.pause(0.01)
            fitness = self.calculate_fitness(self.x,self.x2,TR)
            # 需要更新的个体
            update_id = np.greater(self.individual_best_fitness, fitness)
            self.p[update_id] = self.x[update_id]
            self.p2[update_id] = self.x2[update_id]
            self.individual_best_fitness[update_id] = fitness[update_id]
            # 新一代出现了更小的fitness，所以更新全局最优fitness和位置
            if np.min(fitness) < self.global_best_fitness:
                self.pg = self.x[np.argmin(fitness)]
                self.pg2 = self.x2[np.argmin(fitness)]
                self.global_best_fitness = np.min(fitness)
            print('best fitness: %.5f, mean fitness: %.5f' % (self.global_best_fitness, np.mean(fitness)))


if __name__ == '__main__':
    Gravity = 9.81
    slope = np.zeros(math.floor(1982.0) + 1)
    slope[0:466] = 0
    slope[466:543] = -4
    slope[543:700] = -6
    slope[700:761] = 0
    slope[761:851] = 7
    slope[851:1029]=12
    slope[1029:1115] = 0
    slope[1115:1405] = -2
    slope[1405:1768]=-3
    slope[1768:1802] = -2
    slope[1802:1840]=-1
    slope[1840:-1]=0

    trainmass = 310
    trainlength = 112
    geartable = [1.0,0.8,0.6, 0.4, 0.2, 0., -0.2, -0.4, -0.6, -0.8,-1.0]
    roadlength =  1982
    planningtime = 130
    roadlimited = 80
    stationlimited = 54
    TR = Train_Road(trainmass,trainlength,geartable,roadlength,planningtime,roadlimited,stationlimited,slope,150)
    for i in range(1,11):
        PSO1 = PSO(200,200,TR)
        PSO1.evolve()
        expert_data = np.concatenate(np.arange(1983).reshape(-1,1),TR.TrainSpeed,TR.RunningTime)
        expert_action = TR.Traction
        np.save("expert_data"+str(i),expert_data)
        np.save("expert_action"+str(i),expert_action)
        # tosave = np.array(PSO1.kexj)
        # tosave1 = np.squeeze(tosave)
        # tosave1 = np.unique(tosave1,axis=0)
        # DataFra = pd.DataFrame(tosave1)
        # filename= '1982m-130s-1m-130'+str(i)+'-kexingjie.csv'
        # DataFra.to_csv(filename,index=None)


    # reward =0
    # while True:
    #     act = random.randint(0,7)
    #     print(act)
    #     _,done,t = TR.step(act,steps)
    #     reward+=t
    #     steps+=1
    #     if done:
    #         break
    # x1 = TR.Road
    # x2 = np.arange(0, int(TR.Roadlength)+2, 1)
    # x2[-1] = TR.Totaldistance
    #
    # y2 = TR.RoadSpeedLimited
    # y = TR.TrainSpeed * 3.6
    # y = np.append(y, 0)
    # plt.plot(x2, y)
    # plt.plot(x1, y2)
    # plt.show()
