# plots output of local_search_choice.jl
import os,sys,ast
import numpy as np
import matplotlib.pyplot as plt
plt.rc('font', size=12) 

#param=2
#local_search_mode="1"
#plt.rc('xtick', labelsize=10.5)
input_file=sys.argv[1]#"temp//bar_neighborhoods_"+local_search_mode+"_param"+str(param)+".txt"
output_file=sys.argv[2]#"plots//bar_neighborhoods_"+local_search_mode+"_param"+str(param)+".png"
table_file=sys.argv[3]#"data_files//improvement_table_neighborhoods_"+local_search_mode+"_param"+str(param)+".tex"
cleanup=False#sys.argv[4]=='true'

f=open(input_file)
dic=ast.literal_eval(f.read())
f.close()

species=list(dic.keys())
penguin_means={}
for field in ('smallest obj','longest tour','combo','largest obj','most targets'):
    if field in dic[species[0]]:
        penguin_means[field]=[100*(dic[key][field]/dic[key]['initial']-1) for key in species]
conversion={'smallest obj':"Lowest Objective",
            'largest obj': "Highest Objective",
            'most targets': "Most Targets",
            'longest tour':"Longest Tour",
            'combo':"Combination (Longest Tour + Most Targets)",
            'combo2':"Combination (Lowest + Highest Objective)"
            }
marker_hardly={'smallest obj':"x",'largest obj': "o",'most targets': "D",'longest tour':'+','combo':'*','combo2':'<'}
color_hardly={'smallest obj':"blue",'largest obj': "orange",
              'most targets': "green",'longest tour':'red',
              'combo':'purple','combo2':'magenta'}


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
ax.set_ylabel('Percent Improvement from Initial Solution')
count=9
step=(len(x)-1)/(count-1)
indices=[int(round(i*step)) for i in range(count)]
ax.set_xticks([x[i] for i in indices], [species[i] for i in indices])
ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.3),ncol=2,)

plt.savefig(output_file)


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
if cleanup:
    os.remove(input_file)
