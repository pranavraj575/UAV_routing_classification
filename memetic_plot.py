# plots comparision of objective function of memetic algorithm against our algorithm
# uses output of memetic_eval.jl for memetic values
# uses output of run_experiments.jl for our values

import os,sys,ast
import numpy as np
import matplotlib.pyplot as plt

plt.rc('font', size=12) 
#plt.rc('xtick', labelsize=10.5)



our_files=[
    os.path.join("output","data_files","experiment_results","MD_all_values.txt"),
    ]
meme_files=[
    os.path.join("output","data_files","experiment_results","memetic_results.txt"),
]

output_dir=os.path.join("output","plots", "memetic_comparison")

if not os.path.exists(output_dir):
    os.makedirs(output_dir)
# parameters
# for calculating alpha and tau 
alpha_factors=[1,2]
tau_values=[2.,1.]
# which types of neighborhood to use in local search
local_search_modes=["12","1"]
# check top k results in local search
param_choices=[2,1]
# whether to include outliers in plot
include_outliers=[True]
meme_dic_overall=dict()
our_dic_overall=dict()
for files, dic_to_update in ((meme_files,meme_dic_overall),(our_files,our_dic_overall)):
    for file in files:
        f=open(file)
        dic=ast.literal_eval(f.read())
        f.close()
        for alpha_tau in dic:
            if alpha_tau not in dic_to_update:
                dic_to_update[alpha_tau]=dict()
            for neigh in dic[alpha_tau]:
                if neigh not in dic_to_update[alpha_tau]:
                    dic_to_update[alpha_tau][neigh]=dict()
                dic_to_update[alpha_tau][neigh].update(dic[alpha_tau][neigh])


for alpha in alpha_factors:
    for tau in tau_values:
        for local_search_mode in local_search_modes:
            for param in param_choices:
                try:
                    meme_dic=meme_dic_overall[(alpha,tau)][(local_search_mode,param)]
                    meme_dic={k:meme_dic[k]["final_obj"][0] for k in meme_dic}
                    our_dic=our_dic_overall[(alpha,tau)][(local_search_mode,param)]
                    our_dic={k:our_dic[k]["final_obj"][0] for k in our_dic}
                except:
                    print('keys not in both results:',
                        "alpha_tau",(alpha,tau),
                        "neighborhood",local_search_mode,param)
                    continue
                common_keys=set(our_dic.keys()).intersection(set(meme_dic.keys()))
                if not common_keys:
                    print('no common experiments, parameters:',
                        "alpha_tau",(alpha,tau),
                        "neighborhood",local_search_mode,param)
                    continue

                dic={k:{'memetic':meme_dic[k],'ours':our_dic[k]}for k in common_keys}

                ident="neighborhoods_"+local_search_mode+"_param_"+str(param)+"_alpha_"+str(alpha)+"_tau_"+str(tau)
                for outliers in include_outliers:
                    output_file=os.path.join(output_dir,"memetic_comparision_"+ident+'.png')
                    if outliers:
                        output_file2=os.path.join(output_dir,"memetic_comparison_percentage_"+ident+'.png')
                    else:
                        output_file2=os.path.join(output_dir,"memetic_comparision_percent_no_outliers_"+ident+'.png')


                    species=list(dic.keys())
                    species.sort(key=lambda m:int(m[2:]) if m[2:].isdigit() else 0)
                    penguin_means={}
                    for field in ('memetic','ours'):
                        if field in dic[species[0]]:
                            penguin_means[field]=[dic[key][field] for key in species]
                    conversion={'memetic':"Memetic",
                                'ours': "Our Alg",
                                }
                    marker_hardly={'memetic':"+",
                                'ours': "x"
                                }
                    color_hardly={'memetic':"red",
                                'ours': "blue"}


                    x = np.arange(len(species))  # the label locations
                    width = 0.25  # the width of the bars
                    multiplier = 0

                    fig, ax = plt.subplots(layout='constrained')

                    for attribute, measurement in penguin_means.items():
                        offset = width * multiplier
                        rects = ax.scatter(x, measurement, label=conversion[attribute],alpha=.5,marker=marker_hardly[attribute],s=40,color=color_hardly[attribute])
                        #ax.bar_label(rects, padding=3)
                        multiplier += 1

                    # Add some text for labels, title and custom x-axis tick labels, etc.
                    ax.set_ylabel('Objective Value')
                    count=10
                    step=(len(x)-1)/(count-1)
                    indices=[int(round(i*step)) for i in range(count)]
                    ax.set_xticks([x[i] for i in indices], [species[i] for i in indices])
                    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),ncol=2,)

                    plt.savefig(output_file)
                    plt.close()



                    fig, ax = plt.subplots(layout='constrained')
                    percent_deviations=100*np.array([(dic[key]['ours']-dic[key]['memetic'])/dic[key]['memetic'] for key in species ])
                    while not outliers:
                        done=True
                        for i in range(len(percent_deviations)):
                            if percent_deviations[i]>100:
                                done=False
                                percent_deviations=list(percent_deviations[:i])+list(percent_deviations[i+1:])
                                x=list(x[:i])+list(x[i+1:])
                                break
                        if done:
                            break
                    ax.scatter(x, percent_deviations,marker='*',s=40,color='purple')

                    step=(len(x)-1)/(count-1)
                    indices=[int(round(i*step)) for i in range(count)]
                    ax.set_xticks([x[i] for i in indices], [species[x[i]] for i in indices])
                    plt.ylabel("Percent deviation")
                    plt.plot([0 for _ in x],linestyle='--',alpha=.5,color='green',label="$y=0$")
                    plt.legend()
                    plt.savefig(output_file2)
                    plt.close()
                    print("MEAN of DEVIATIONS:\t",np.mean(percent_deviations),sep='')
                    print("MEDIAN of DEVIATIONS:\t",np.median(percent_deviations),sep='')
                    print("STD of DEVIATIONS:\t",np.std(percent_deviations),sep='')
                    print("RANGE of DEVIATIONS:\t","[",np.min(percent_deviations),', ',np.max(percent_deviations),"]",sep='')




quit()



f=open(table_file,'w')

f.write("\\begin{table}[htb!]\n")
f.write("\t\\centering\n")
f.write("\t\\begin{tabular}{|c|c|c|c|c|}\n")
f.write("\t\\hline\n")
f.write("\t\\textbf{Method} ")
f.write("& \\textbf{Minimum} ")
f.write("& \\textbf{Median} ")
f.write("& \\textbf{Mean} ")
f.write("& \\textbf{Maximum} ")
f.write("\\\\\\hline\n")

for field in penguin_means:
    f.write("\t")
    f.write("\\textit\\textbf{{")
    f.write(conversion[field])
    f.write("}}")
    f.write(" & ")
    f.write((f"{np.min(penguin_means[field]):.2e}"))
    f.write(" & ")
    f.write(str(round(np.median(penguin_means[field]),2)))
    f.write(" & ")
    f.write(str(round(np.mean(penguin_means[field]),2)))
    f.write(" & ")
    f.write(str(round(np.max(penguin_means[field]),2)))
    f.write("\\\\\\hline\n")



f.write("\t\\end{tabular}\n")

f.write("\\caption{Percentage improvement of Local Search by heuristic for \\lq vehicle to remove target from\\rq}\n")
f.write("\\label{tab:percentage_improvement}\n")
f.write("\\end{table}")
f.close()
