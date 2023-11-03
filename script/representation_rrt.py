#! /usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

rrred = '#ff2222'



fig, ax = plt.subplots(2, 3)

success_rate = np.zeros((4,6))

quant = [0.02, 0.25, 0.75, 0.98]

# Helper functions
def format_func(value, tick_number):
    if value == 1:
        return "Parabol"
    else:
        return "Naive"

def setViolinFormat(parts, data, ax):
    for pc in parts['bodies']:
#        pc.set_facecolor('#D43F3A')
        pc.set_edgecolor('black')
        pc.set_alpha(1)

    perc = [2, 25, 50, 75, 98]
    min_, quartile1, medians, quartile3, max_ = np.percentile(data, perc, axis=1)

    inds = np.arange(1, len(medians) + 1)
    ax.scatter(inds, medians, marker='o', color='white', s=30, zorder=3)
    ax.vlines(inds, quartile1, quartile3, color='k', linestyle='-', lw=5)
    ax.vlines(inds, min_, max_, color='k', linestyle='-', lw=1)

    ax.set_ylim([0, max(max_) * 1.05])

# Start representation
for i in range(1,7):
    if i==6:
        i=7

    # Load results
    data_bis = np.loadtxt('rrt_stats_bisection_'+str(i)+'.txt')
    data_par = np.loadtxt('rrt_stats_parable_'+str(i)+'.txt')
    if i==7:
        i=6

    # Get the results of all
    data = [ data_par[:,0], data_bis[:,0]]
    curr_ax = ax[int((i-1)/3), (i-1)%3]



    violin_parts = curr_ax.violinplot(data, showmeans=False, showmedians=False,
                                      showextrema=False)
#    median_ = violin_parts['cmedians']
#    median_.set_edgecolor(rrred)
    curr_ax.set_title('Scenario '+ str(i), fontsize=16)
#    curr_ax.set_xlabel("Method", fontsize=16)
    curr_ax.set_ylabel("Execution time (s)", fontsize=16)
    curr_ax.ticklabel_format(axis='both', style='sci', useMathText=True, scilimits=(-2,2))
    curr_ax.tick_params(labelsize=14)
    curr_ax.set_xticks([1,2])
#    curr_ax.xaxis.set_label_coords(.5, -.025)

    curr_ax.xaxis.set_major_formatter(plt.FuncFormatter(format_func))
    setViolinFormat(violin_parts, data, curr_ax)

    success_rate[0,i-1] = np.count_nonzero(data_par[:,2] == 1) / len(data_par[:,1])
    success_rate[1,i-1] = np.count_nonzero(data_bis[:,2] == 1) / len(data_bis[:,1])


figManager = plt.get_current_fig_manager()
figManager.window.attributes('-fullscreen', True)
fig.set_size_inches(16.5, 10.5)
plt.savefig('/home/muten/paper_marsupial/git/Images/results_rrt_2.png', bbox_inches='tight')
plt.show()

# Cluttered ones
fig, ax = plt.subplots(2, 3)

for i in range(1,7):
    if i==6:
        i=7
    data_bis = np.loadtxt('rrt_stats_bisection_'+str(i)+'_2.txt')
    data_par = np.loadtxt('rrt_stats_parable_'+str(i)+'_2.txt')
    if i==7:
        i=6

    # Get the results of all
    data = [ data_par[:,0], data_bis[:,0]]
    curr_ax = ax[int((i-1)/3), (i-1)%3]

    violin_parts = curr_ax.violinplot(data, showmeans=False, showmedians=False, 
                                      showextrema = False)
#    median_ = violin_parts['cmedians']
#    median_.set_edgecolor(rrred)
    curr_ax.set_title('Scenario '+ str(i), fontsize=16)
    curr_ax.set_xlabel("Method", fontsize=16)
    curr_ax.set_ylabel("Execution time (s)", fontsize=16)
    curr_ax.ticklabel_format(axis='both', style='sci', useMathText=True, scilimits=(-2,2))
    curr_ax.tick_params(labelsize=14)
    curr_ax.set_xticks([1,2])
    curr_ax.xaxis.set_label_coords(.5, -.025)

    success_rate[2,i-1] = np.count_nonzero(data_par[:,2] == 1) / len(data_par[:,1])
    success_rate[3,i-1] = np.count_nonzero(data_bis[:,2] == 1) / len(data_bis[:,1])

    curr_ax.xaxis.set_major_formatter(plt.FuncFormatter(format_func))

    setViolinFormat(violin_parts, data, curr_ax)


print (success_rate)

figManager = plt.get_current_fig_manager()
figManager.window.attributes('-fullscreen', True)
fig.set_size_inches(16.5, 10.5)
plt.savefig('/home/muten/paper_marsupial/git/Images/results_rrt_2.png', bbox_inches='tight')
plt.show()




