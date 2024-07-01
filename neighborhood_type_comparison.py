"""
To be run after run_experiment.jl
"""
import ast, numpy as np, os
from matplotlib import pyplot as plt

plt.rc('font', size=12) 

# files to grab dictionary from
files=[os.path.join("output","data_files","experiment_results","MD_all_values.txt"),] 

# path to save plots to
plot_dir=os.path.join("output","plots","neighborhood_type_comparison")

if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)

dic=dict()
for filee in files:
    if os.path.exists(filee):
        r=open(filee)
        dic.update(ast.literal_eval(r.read()))
        r.close()
index_dict={}
y_ax_conversion={
    'percent_improvements':'% Improvement from Initial',
    'compute_times':'Computation Time'
}
def index(alpha_factor,tau,field,key):
    global index_dict
    dicc=dic[(alpha_factor,tau)][field][key]
    percents=dicc['percent_improvements']
    times=dicc['compute_times']

    if (alpha_factor,tau,field,key) not in index_dict:
        index_dict[(alpha_factor,tau,field,key)]=max(list(range(len(percents))),key=lambda i:percents[i])
    return index_dict[(alpha_factor,tau,field,key)]
    
for plotting in ('percent_improvements','compute_times'):
    print(plotting)
    for alpha_factor,tau in dic:
        print("\talpha:",alpha_factor,"; tau:",tau)
        d=dic[(alpha_factor,tau)]
        fields=(("1",1),("1",2),("12",1),("12",2))
        penguin_means={}
        for field in fields:
            if field in d:
                species=list(d[field].keys())
                species.sort(key=lambda s:int(s[2:]))
                penguin_means[field]=[d[field][key][plotting][index(alpha_factor,tau,field,key)] for key in species]
                print("\t\tfield:",field)
                minn=round(np.min(penguin_means[field]),2)
                if minn==0:
                    minn="{:.2e}".format(np.min(penguin_means[field]))
                print("\t\t\t"+str(minn),
                                    round(np.median(penguin_means[field]),2),
                                    round(np.mean(penguin_means[field]),2),
                                    round(np.max(penguin_means[field]),2),sep=' & ')
                #print("\t\t\tMIN:",np.min(penguin_means[field]))
                #print("\t\t\tMEDIAN:",np.median(penguin_means[field]))
                #print("\t\t\tMEAN:",np.mean(penguin_means[field]))
                #print("\t\t\tMAX:",np.max(penguin_means[field]))

        conversion={("1",1):"1 pt. move; top choice",
                    ("1",2):"1 pt. move; top 2 choices",
                    ("12",1):"1 pt. move/swap; top choice",
                    ("12",2):"1 pt. move/swap; top 2 choices",
                }
        marker_hardly={("1",1):"o",
                    ("1",2):"x",
                    ("12",1):"+",
                    ("12",2):"*",
                }
        
        color_hardly={("1",1):"green",
                    ("1",2):"blue",
                    ("12",1):"red",
                    ("12",2):"purple",
                }
        
        x = np.arange(len(species))  # the label locations
        width = 0.25  # the width of the bars
        multiplier = 0

        fig, ax = plt.subplots(layout='constrained')

        for attribute, measurement in penguin_means.items():
            offset = width * multiplier
            rects = ax.scatter(x, measurement, label=conversion[attribute],alpha=.5,marker=marker_hardly[attribute],s=40,color=color_hardly[attribute])
            multiplier += 1
        ax.set_ylabel(y_ax_conversion[plotting])
        count=10
        step=(len(x)-1)/(count-1)
        indices=[int(round(i*step)) for i in range(count)]
        ax.set_xticks([x[i] for i in indices], [species[i] for i in indices])
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.2),ncol=2,)

        plot_name=os.path.join(plot_dir,
                                "local_search_"+plotting+"_comparision_alpha_factor_"+str(alpha_factor)+"_tau_"+str(tau)+".png")
        plt.savefig(plot_name, bbox_inches='tight')
        plt.close()
