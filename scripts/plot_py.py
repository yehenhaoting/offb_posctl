#encoding: utf-8
import csv
import numpy as np
import matplotlib.pyplot as plt
with open("/home/zm/catkin_ws/src/offb_posctl/src/log031102.csv","r") as file:  #modify the file directory here
    r = csv.reader(file)

    Time,TargetPosition_x,TargetPosition_y,TargetPosition_z,RealPosition_x,RealPosition_y,RealPosition_z,TargetVel_x,TargetVel_y,TargetVel_z,RealVel_x,RealVel_y,RealVel_z,TargetAngle_x,TargetAngle_y,TargetAngle_z,RealAngle_x,RealAngle_y,RealAngle_z, RealAcc_x, RealAcc_y, RealAcc_z, TargetThrust = [],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
    index = 0
    for i in  r :
        if(index !=  0 ):
            Time.append(i[0])

            TargetPosition_x.append(i[1])
            TargetPosition_y.append(i[2])
            TargetPosition_z.append(i[3])
            RealPosition_x.append(i[4])
            RealPosition_y.append(i[5])
            RealPosition_z.append(i[6])

            TargetVel_x.append(i[7])
            TargetVel_y.append(i[8])
            TargetVel_z.append(i[9])
            RealVel_x.append(i[10])
            RealVel_y.append(i[11])
            RealVel_z.append(i[12])

            TargetAngle_x.append(i[13])
            TargetAngle_y.append(i[14])
            TargetAngle_z.append(i[15])
            RealAngle_x.append(i[16])
            RealAngle_y.append(i[17])
            RealAngle_z.append(i[18])

            RealAcc_x.append(i[19])
            RealAcc_y.append(i[20])
            RealAcc_z.append(i[21])

            TargetThrust.append(i[22])
        #print(i)
        index =index+1
    list = ['TargetPosition_x', 'TargetPosition_y', 'TargetPosition_z', 'RealPosition_x', 'RealPosition_y', 
    'RealPosition_z', 'TargetVel_x','TargetVel_y', 'TargetVel_z', 'RealVel_x','RealVel_y', 'RealVel_z',
    'TargetAngle_x', 'TargetAngle_y','TargetAngle_z','RealAngle_x', 'RealAngle_y','RealAngle_z','RealAcc_x','RealAcc_y','RealAcc_z','TargetThrust']
    # lists = {};
    #lists["Date"],lists["Open"],lists["High"],lists["Low"],lists["Close"],lists["Volume"],lists["Adj_Close"] = Date,Open,High,Low,Close,Volume,Adj_Close
    # lists["Open"],lists["High"],lists["Low"],lists["Close"],lists["Volume"],lists["Adj_Close"] = TargetPosition_x,High,Low,Close,Volume,Adj_Close
#print(lists)
"""  制图开始   """
# list1 = ['-', '_', 'v', '-.', ':', ':']
colors = ['r','g','r','g','r','g','r','g','r','g','r','g','r','g','r','g','r','g','r','g','r','g','r']
"""开始画图"""
# fit =plt.figure()
#组装 legends 对象fl
# legends = {}
# for i in range(len(list)):
#     #legends[list[i]]= list1[i]
# print(legends)

# x = [x for x in range(len(lists["Open"]))]
# for index,t in  enumerate(list):#迭代
#     #print(index,t,list[index])
#     fit.add_subplot("61%s"%(index + 1 ))#subplot 页面布局
#     plt.plot(Time,lists[list[index]],color = colors[index])#填充数据（1.x轴数据，2,.y轴数据，3.线条，4.颜色）
#     plt.legend(t,loc ="upper left" )
# plt.show()

# # figure 1
# plt.figure()
# plt.subplot(131)
# plt.title('TargetPosition_x-RealPosition_x')
# plt.plot(Time, TargetPosition_x,color='r',label='TargetPosition_x')
# plt.plot(Time, RealPosition_x,color='b',label='RealPosition_x')
# plt.legend(loc='lower left')
#
# plt.subplot(132)
# plt.title('TargetPosition_y-RealPosition_y')
# plt.plot(Time, TargetPosition_y ,color='r',label='TargetPosition_y')
# plt.plot(Time, RealPosition_y,color='b',label='RealPosition_y')
# plt.legend(loc='lower left')
#
# plt.subplot(133)
# plt.title('TargetPosition_z-RealPosition_z')
# plt.plot(Time, TargetPosition_z,color='r',label='TargetPosition_z')
# plt.plot(Time, RealPosition_z,color='b',label='RealPosition_z')
# plt.legend(loc='lower left')
#
# # figure 2
# plt.figure()
# plt.subplot(131)
# plt.title('TargetVel_x-RealVel_x')
# plt.plot(Time, TargetVel_x,color='r',label='TargetVel_x')
# plt.plot(Time, RealVel_x,color='b',label='RealVel_x')
# plt.legend(loc='lower left')
#
# plt.subplot(132)
# plt.title('TargetVel_y-RealVel_y')
# plt.plot(Time, TargetVel_y,color='r',label='TargetVel_y')
# plt.plot(Time, RealVel_y,color='b',label='RealVel_y')
# plt.legend(loc='lower left')
#
# plt.subplot(133)
# plt.title('TargetVel_z-RealVel_z')
# plt.plot(Time, TargetVel_z,color='r',label='TargetVel_z')
# plt.plot(Time, RealVel_z,color='b',label='RealVel_z')
# plt.legend(loc='lower left')
#
# # figure 3
# plt.figure()
# plt.subplot(131)
# plt.title('TargetAngle_x-RealAngle_x')
# plt.plot(Time, TargetAngle_x,color='r',label='TargetAngle_x')
# plt.plot(Time, RealAngle_x,color='b',label='RealAngle_x')
# plt.legend(loc='lower left')
#
# plt.subplot(132)
# plt.title('TargetAngle_y-RealAngle_y')
# plt.plot(Time, TargetAngle_y,color='r',label='TargetAngle_y')
# plt.plot(Time, RealAngle_y,color='b',label='RealAngle_y')
# plt.legend(loc='lower left')
#
# plt.subplot(133)
# plt.title('TargetAngle_z-RealAngle_z')
# plt.plot(Time, TargetAngle_z,color='r',label='TargetAngle_z')
# plt.plot(Time, RealAngle_z,color='b',label='RealAngle_z')
# plt.legend(loc='lower left')
#
# # figure 4
# plt.figure()
# plt.title('Thrust')
# plt.plot(Time, TargetThrust,color='r',label='Thrust')

# figure 5
plt.figure()
plt.title('Acc_x')
plt.plot(Time, RealAcc_x,color='r',label='RealAcc_x')

# figure 6
plt.figure()
plt.title('Acc_y')
plt.plot(Time, RealAcc_y,color='r',label='RealAcc_y')

# figure 7
plt.figure()
plt.title('Acc_z')
plt.plot(Time, RealAcc_z,color='r',label='RealAcc_z')

plt.show()
