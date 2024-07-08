# uses networkx to graph the result of a tour solution
# called by network_graphing.jl

import os,sys,ast
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from itertools import combinations
from src.route_graphing_utils import graph_route

input_file=sys.argv[1]
output_file=sys.argv[2]
cleanup=sys.argv[3]=='true'

depot_color="red"
target_color="#00b4d9"

f=open(input_file)
dic=ast.literal_eval(f.read())
f.close()


color_vectors=[np.array([float(i==k) for k in range(3)]) for i in range(3)]

def rand_color():
    v=np.array(color_vectors)@np.random.random(3)
    return v/np.linalg.norm(v)

# colors of paths
color_list=['red','green','blue','cyan','magenta','yellow']

# increase the line width and font size
node_size=300
node_size=100/(1+(len(dic['targets'])+len(dic['depots']))//100)

# edge size
edge_width=1

# Graph does not have arrows, DiGraph does
# G = nx.Graph()
G = nx.DiGraph()

# axis fontsize
axis_fontsize=20

ax = plt.gca()

tour_keys=sorted(list(dic['tours'].keys()))

tour_labels=tour_labels=["vhcl "+str(tour_no)+" avg. $d_i$: "+str(round(dic['dwell_times'][tour_no],2))
                                    for tour_no in tour_keys]

graph_route(targets=np.array([dic['targets'][key] for key in sorted(list(dic['targets'].keys()))]),
                depots=np.array([dic['depots'][key] for key in sorted(list(dic['depots'].keys()))]),
                tours=[dic['tours'][tour_no] for tour_no in tour_keys],
                tour_labels=tour_labels,
                with_labels=False,
                color_list=color_list,
                rotation=None,
                depot_color=depot_color,
                target_color=target_color,
                node_size=node_size,
                edge_width=edge_width,
                random_seed=None,
                ax=ax,
                )
ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
limits=plt.axis('on') 

ax.xaxis.label.set_fontsize(axis_fontsize)
ax.yaxis.label.set_fontsize(axis_fontsize)
for item in ax.get_xticklabels()+ ax.get_yticklabels():
    item.set_fontsize(15)

plt.rc('font', size=15) 
legend_height=1.05+.1*(np.ceil(len(dic['depots'])/3))

# comment out to remove legend
plt.legend(loc='upper center', bbox_to_anchor=(.5, legend_height),ncol=3)

# x and y axes
plt.xlabel("X (m)");plt.ylabel("Y (m)")

plt.savefig(output_file, bbox_inches='tight')
if cleanup:
    os.remove(input_file)

