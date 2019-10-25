#encoding: utf-8
import csv
import numpy as np
import matplotlib.pyplot as plt
with open("/home/zm/catkin_ws/src/offb_posctl/src/debug031101.csv","r") as file:  #modify the file directory here
    r = csv.reader(file)

    Time,FilterIn_x, FilterIn_y, FilterOut_x, FilterOut_y, FilterOut01_x, FilterOut01_y = [],[],[],[],[],[],[]
    index = 0
    for i in  r :
        if(index !=  0 ):
            Time.append(i[0])

            FilterIn_x.append(i[1])
            FilterIn_y.append(i[2])
            FilterOut_x.append(i[3])
            FilterOut_y.append(i[4])
            FilterOut01_x.append(i[5])
            FilterOut01_y.append(i[6])

        index =index+1
    list = ['FilterIn_x', 'FilterIn_y', 'FilterOut_x', 'FilterOut_y', 'FilterOut01_x', 'FilterOut01_y']
    # lists = {};
    #lists["Date"],lists["Open"],lists["High"],lists["Low"],lists["Close"],lists["Volume"],lists["Adj_Close"] = Date,Open,High,Low,Close,Volume,Adj_Close
    # lists["Open"],lists["High"],lists["Low"],lists["Close"],lists["Volume"],lists["Adj_Close"] = TargetPosition_x,High,Low,Close,Volume,Adj_Close
#print(lists)
"""  制图开始   """
# list1 = ['-', '_', 'v', '-.', ':', ':']
colors = ['r','g','r','g','r','g','r']
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

# figure 1
plt.figure()
# plt.subplot(131)
plt.title('FilterIn_Out_x')
plt.plot(Time, FilterIn_x,color='r',label='FilterIn_x')
plt.plot(Time, FilterOut_x,color='b',label='FilterOut_x')
plt.plot(Time, FilterOut01_x,color='g',label='FilterOut01_x')
plt.legend(loc='lower left')


plt.figure()
# plt.subplot(132)
plt.title('FilterIn_Out_y')
plt.plot(Time, FilterIn_y ,color='r',label='FilterIn_y')
plt.plot(Time, FilterOut_y,color='b',label='FilterOut_y')
plt.plot(Time, FilterOut01_y,color='g',label='FilterOut01_y')
plt.legend(loc='lower left')

plt.show()
