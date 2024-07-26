#! /usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

def format_func(value, tick_number):
    if value == 1:
        return "Parabol"
    else:
        return "Naive"



rrred = '#ff2222'

def setViolinFormat(parts, data, ax):
    for pc in parts['bodies']:
#        pc.set_facecolor('#D43F3A')
        pc.set_edgecolor('black')
        pc.set_alpha(1)

    perc = [2, 25, 50, 75, 98]
    min_, quartile1, medians, quartile3, max_ = np.percentile(data.transpose(), perc, axis=1)
    print("Medians: ", medians)

    inds = np.arange(1, len(medians) + 1)
    ax.scatter(inds, medians, marker='o', color='white', s=30, zorder=3)
    ax.vlines(inds, quartile1, quartile3, color='k', linestyle='-', lw=5)
    ax.vlines(inds, min_, max_, color='k', linestyle='-', lw=1)

    ax.set_ylim([0, max(max_) * 1.05])


# Load results
fig, ax = plt.subplots(2, 3)
for i in range(1,7):
    if i==6:
        i=7

    data = np.loadtxt('catenary_stats_'+str(i)+'.txt', skiprows=1)
    d = data[:, [0,2]]
    if i==7:
        i=6


    curr_ax = ax[int((i-1)/3), (i-1)%3]

    violin_parts = curr_ax.violinplot(data[:, [0,2]], showmeans=False, showextrema=False, showmedians=False, widths = 1)

    
#    vp = violin_parts['cmedians']
#    vp.set_edgecolor(rrred)

    curr_ax.set_title('Scenario '+ str(i), fontsize=16)
#    curr_ax.set_xlabel("Method", fontsize=16)
    curr_ax.set_ylabel("Execution time (s)", fontsize=16)
    curr_ax.ticklabel_format(axis='both', style='sci', useMathText=True, scilimits=(-2,2))
    curr_ax.tick_params(labelsize=14)
    curr_ax.set_xticks([1,2])
#    curr_ax.xaxis.set_label_coords(.5, -.025)
    curr_ax.xaxis.set_major_formatter(plt.FuncFormatter(format_func))

    print (d.shape)
    setViolinFormat(violin_parts, d, curr_ax)


figManager = plt.get_current_fig_manager()
figManager.window.attributes('-fullscreen', True)
fig.set_size_inches(17, 10.5)
plt.savefig('/home/muten/paper_marsupial/git/Images/results_1.png', bbox_inches='tight')
plt.show()

fig, ax = plt.subplots(2, 3)
for i in range(1,7):
    data = np.loadtxt('catenary_stats_'+str(i)+'_2.txt', skiprows=1)

    curr_ax = ax[int((i-1)/3), (i-1)%3]

    violin_parts = curr_ax.violinplot(data[:, [0,2]], positions=[1, 2], showmeans=False,
                                      showextrema=False,
                                      showmedians=False)
#    vp = violin_parts['cmedians']
#    vp.set_edgecolor(rrred)

    curr_ax.set_title('Scenario '+ str(i),fontsize=16)
#    curr_ax.set_xlabel("Method", fontsize=16)
    curr_ax.set_ylabel("Execution time (s)", fontsize=16)
    curr_ax.ticklabel_format(axis='both', style='sci', useMathText=True, scilimits=(-2,2))
    curr_ax.tick_params(labelsize=14)
    curr_ax.set_xticks([1,2])
#    curr_ax.xaxis.set_label_coords(.5, -.025)

    curr_ax.xaxis.set_major_formatter(plt.FuncFormatter(format_func))
    setViolinFormat(violin_parts, data[:, [0,2]], curr_ax)

figManager = plt.get_current_fig_manager()
figManager.window.attributes('-fullscreen', True)
fig.set_size_inches(17, 10.5)
plt.savefig('/home/muten/paper_marsupial/git/Images/results_2.png', bbox_inches='tight')
plt.show()

