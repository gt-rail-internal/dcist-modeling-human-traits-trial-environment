from matplotlib import pyplot as plt
from matplotlib.patches import PathPatch
from matplotlib import cbook
import scipy.stats
import itertools
import numpy
import seaborn
import pandas as pd
import pickle
import functools
import operator
import statistics
from os.path import exists

# utility function for nicer box plots
def adjust_box_widths(g, fac):
    """
    Adjust the widths of a seaborn-generated boxplot.
    """

    # iterating through Axes instances
    for ax in g.axes:

        # iterating through axes artists:
        for c in ax.get_children():

            # searching for PathPatches
            if isinstance(c, PathPatch):
                # getting current width of box:
                p = c.get_path()
                verts = p.vertices
                verts_sub = verts[:-1]
                xmin = numpy.min(verts_sub[:, 0])
                xmax = numpy.max(verts_sub[:, 0])
                xmid = 0.5*(xmin+xmax)
                xhalf = 0.5*(xmax - xmin)

                # setting new width of box
                xmin_new = xmid-fac*xhalf
                xmax_new = xmid+fac*xhalf
                verts_sub[verts_sub[:, 0] == xmin, 0] = xmin_new
                verts_sub[verts_sub[:, 0] == xmax, 0] = xmax_new

                # setting new width of median line
                for l in ax.lines:
                    if numpy.all(l.get_xdata() == [xmin, xmax]):
                        l.set_xdata([xmin_new, xmax_new])


# plots the n choose 3 plots
def plot_nc3():
    process_data_names = []  # data items to process into the df arrays
    score_data_names = ["3c3", "4c3", "5c3", "6c3", "7c3"]#, "7c3"]#, "8c3"]

    #score_dists = []  # score distribution of all scores

    df_group_size = []
    df_team_scores = []
    df_assignment_type = []

    # if exists("df_small_nc3.pkl"):
    #     print("DF file exists, loading")
    #     data = pickle.load(open("df_small_nc3.pkl", "rb"))
    #     df_group_size = data[0]
    #     df_team_scores = data[1]
    #     df_assignment_type = data[2]
    # else:
    #     process_data_names = score_data_names

    #total_score_data = {}
    
    # for each data set
    saved_data = {}
    for name in score_data_names:
        print("Processing", name)
        score_data = pickle.load(open("score_data_" + name + ".pkl", "rb"))  # load the score data
        #total_score_data[name] = {}
        #best_based = [x[0] for x in score_data]  # pull the data
        data_types = ["best", "worst", "pred weak", "Individualized Role Assignment", "something", "something", "Random Assignment"]
        #saved_data = {}
        for i in [3, 6]:  # 3:ability, 6:random
            data = [x[i] for x in score_data]  # pull the data
            if i == 6:  # flatten if random
                data = functools.reduce(operator.iconcat, data, [])

            #print(">>>", name, i)

            # boxplot stats
            data = sorted(data)  # sort from small to large
            saved_data[name[0] + ":" + str(i)] = data

            print("length", len(data))
            min_val = data[0]  # low
            max_val = data[-1]  # high
            num_items = len(data)
            #print("num items", num_items)
            even = False if num_items % 2 else True

            q1_val = data[1 * num_items // 4]  # Q1
            q2_val = data[2 * num_items // 4]  # median
            q3_val = data[3 * num_items // 4]  # Q3

            if even:  # if even, average two values
                q1_val = (q1_val + data[1 * num_items // 4 + 1]) / 2
                q2_val = (q2_val + data[2 * num_items // 4 + 1]) / 2
                q3_val = (q3_val + data[3 * num_items // 4 + 1]) / 2

            df_group_size.extend([int(name[0])] * 5)  # add N 
            df_team_scores.extend([min_val, q1_val, q2_val, q3_val, max_val])  # add data
            print(">>>", name, data_types[i], [min_val, q1_val, q2_val, q3_val, max_val], "avg", sum(data) / len(data))
            df_assignment_type.extend([data_types[i]] * 5)  # add assignment type label
    #print(">>>", saved_data["3:3"])
    wilcox = scipy.stats.mannwhitneyu(saved_data["3:3"], saved_data["4:3"], alternative="greater")
    print("   Wilcoxan 3 4", wilcox)
    wilcox = scipy.stats.mannwhitneyu(saved_data["3:3"], saved_data["5:3"], alternative="greater")
    print("   Wilcoxan 3 5", wilcox)
    wilcox = scipy.stats.mannwhitneyu(saved_data["3:3"], saved_data["6:3"], alternative="greater")
    print("   Wilcoxan 3 6", wilcox)
    wilcox = scipy.stats.mannwhitneyu(saved_data["4:3"], saved_data["6:3"], alternative="greater")
    print("   Wilcoxan 4 6", wilcox)

        # stats
        #count_best = 0
        #count_worst = 0
        #count_better_random = 0
        #count_better_avg = 0
        #count_samples = 0
        #count_total = 0
        #sum_total = 0
        #raw_rand_scores = {}
        #print("Processing stats")
        #for x in score_data:
        #    count_samples += 1
        #    # against best
        #    # if x[3] == x[0]:
        #    #    count_best += 1
        #    # if x[3] == x[4]:
        #    #    count_worst += 1
        #    # count_better_random += len([y for y in x[-1] if x[3] >= y])
        #    # sum_total += sum(x[-1])
        #    # avg = sum([y for y in x[-1]]) / len(x[-1])
        #    # if x[3] > avg:
        #    #    count_better_avg += 1
        #    # count_total += len(x[-1])

        #    # for y in x[-1]:
        #    #    if y not in raw_rand_scores:
        #    #        raw_rand_scores[y] = 0
        #    #    raw_rand_scores[y] += 1

        #print("Stats:", "outperform ratio", round(count_better_avg / count_samples, 3))
        #print("scores:", raw_rand_scores)
        #print(">>>>>", raw_rand_scores[1.0014500856549562], raw_rand_scores[1.5291057647223698], raw_rand_scores[0.4705633624554732])

        # add to the group
        #df_group_size.extend([int(name[0])] * len(ability_based))  # add ability based to group size
        #df_team_scores.extend(ability_based)  # add ability based scores to team scores
        #df_assignment_type.extend(["Skill-Based Assignment"] * len(ability_based))  # add ability based label to assignment type

        #df_group_size.extend([int(name[0])] * len(random_based_flat))  # add random based to group size
        #df_team_scores.extend(random_based_flat)  # add random based scores to team scores
        #df_assignment_type.extend(["Random Assignment"] * len(random_based_flat))  # add random based label to assignment type

    data = pd.DataFrame({"Size of Group (N)": df_group_size,
                         "Team Scores by Assignment Type": df_team_scores,
                         "Assignment Type": df_assignment_type,
                         })
    
    
    # generate violin plot
    #print("Generating violin")
    #ax = seaborn.violinplot(data=data, x="Team Scores by Assignment Type", y="Size of Group (n choose 3)", hue="Assignment Type", orient="h", inner=None, split=True, palette="pastel", fontsize=20)
    ax = None
    #print("Finished violin")


    pickle.dump([df_group_size, df_team_scores, df_assignment_type], open("df_small_nc3.pkl", "wb"))

        #stats[name] = [ability_min_val, ability_q1_val, ability_q2_val, ability_q3_val, ability_max_val]

    data_small = pd.DataFrame({"Size of Group (N)": df_group_size,
                        "Team Scores by Assignment Type": df_team_scores,
                        "Assignment Type": df_assignment_type,
                        })

    #print(">>>", data_small)

    #plt.boxplot([stats[name] for name in score_data_names], positions=[int(name[0]) for name in score_data_names], vert=False)
    #ax = seaborn.boxplot(data=data_small, y='Team Scores by Assignment Type', x='Size of Group (n choose 3)', hue='Assignment Type')
    if ax is None:  # create new
        ax = seaborn.boxplot(data=data_small, y='Team Scores by Assignment Type', x='Size of Group (N)', hue='Assignment Type', saturation=0.5, width=0.5,
                            boxprops={'zorder': 2}, whis=100, palette="Blues")
        ax.set_ylim([0,3.4])
        ax.set_xlabel("Size of Group (N)", fontsize=25)
        plt.xticks(fontsize=20)
        ax.set_ylabel("Cumulative Performance Score", fontsize=25)
        plt.yticks(fontsize=20)
        plt.legend(fontsize=20, loc="upper right", frameon=False)

    else: # build upon violin plot
        ax = seaborn.boxplot(data=data_small, y='Team Scores by Assignment Type', x='Size of Group (N)', hue='Assignment Type', saturation=0.5, width=0.3,
            boxprops={'zorder': 2}, ax=ax, showfliers=False, whis=0)
    
        for box in ax.artists:
            box.set_facecolor("grey")
            r, g, b, a = box.get_facecolor()
            box.set_facecolor((r, g, b, 0.5))
        seaborn.boxplot(data=data, x="Team Scores by Assignment Type", y="Size of Group (N)", hue="Assignment Type")

    adjust_box_widths(ax.get_figure(), 0.9)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)

    
    plt.show()

    
plot_nc3()