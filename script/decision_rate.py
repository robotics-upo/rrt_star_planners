#! /usr/bin/python3
#! This script is used to calculate the rates for the paper
import numpy as np


# Load results

for i in range(1,6):
    for j in range(1,3):
        name = 'catenary_stats_'+str(i)+'_'+str(j)+'.txt'
        data = np.loadtxt(name, skiprows=1)
        straight_cont = 0
        fail_parabola = 0
        fail_catenary = 0
        total = len(data)

        for k in range(len(data)):
            curr_data = data[k, :]
            if curr_data[-1] == 1:
                straight_cont += 1
            if curr_data[-2] == -1:
                fail_catenary += 1
            if curr_data[1] == -1:
                fail_parabola +=1
        
        print(name)
        print('Total', total, 'Straight cont', straight_cont)
        print('DP rate ', 1 - float(straight_cont)/float(total))
        print('Success rate Parabola ', 1 - float(fail_parabola)/float(total))
        print('Success rate Catenary ', 1 - float(fail_catenary)/float(total))


        print('')





