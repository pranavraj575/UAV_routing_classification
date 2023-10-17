import os,sys,ast
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from itertools import combinations

input_file=sys.argv[1]
output_file=sys.argv[2]
cleanup=sys.argv[3]=='true'

depot_color="black"
target_color="#00b4d9"

f=open(input_file)
dic=ast.literal_eval(f.read())
f.close()


color_vectors=[np.array([float(i==k) for k in range(3)]) for i in range(3)]

def rand_color(brightness=.25):
    v=np.zeros(3)
    while np.linalg.norm(v)<brightness:
        v=np.array(color_vectors)@np.random.random(3)
    return v

color_list=[rand_color(), 'red','green','blue','cyan','magenta','yellow']+[rand_color() for _ in range(len(dic['depots']))]

node_size=300#100/(1+(len(dic['targets'])+len(dic['depots']))//100)
# increase the line width and font size
G = nx.Graph()
ax = plt.gca()

for target_key in dic['targets']:
    G.add_node(target_key,pos=dic['targets'][target_key],color=target_color)
for depot_key in dic['depots']:
    G.add_node(depot_key,pos=dic['depots'][depot_key],color=depot_color)

for tour_no in dic['tours']:
    tour=dic['tours'][tour_no]
    color=color_list[tour_no%len(color_list)]
    for i in range(len(tour)-1):
        if i==0:
            label="vhcl "+str(tour_no)+" avg. $d_i$: "+str(round(dic['dwell_times'][tour_no],2))
            ax.plot([0],[0],
            color=color,
            label=label,
            linewidth=4)
        G.add_edge(tour[i],tour[i+1],color=color,width=4)
        

node_color = list(nx.get_node_attributes(G,'color').values())
edge_color = list(nx.get_edge_attributes(G,'color').values())
edge_widths = list(nx.get_edge_attributes(G,'width').values())
pos=(nx.get_node_attributes(G,"pos"))

nx.draw_networkx(G,
         pos=pos,
        with_labels=True, 
        font_weight='bold', 
        node_color=node_color,
        edge_color=edge_color,
        node_size=node_size,
        width=edge_widths,
        ax=ax)
ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
ax.xaxis.label.set_fontsize(20)
ax.yaxis.label.set_fontsize(20)
for item in ax.get_xticklabels()+ ax.get_yticklabels():
    item.set_fontsize(15)
limits=plt.axis('on') 
plt.rc('font', size=15) 
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
#ax.legend(loc='center left', bbox_to_anchor=(1.5, 0),ncol=1,)
#plt.legend(loc='center left', bbox_to_anchor=(1, .5),ncol=5)
legend_height=1.05+.1*(np.ceil(len(dic['depots'])/3))
plt.legend(loc='upper center', bbox_to_anchor=(.5, legend_height),ncol=3)

plt.savefig(output_file, bbox_inches='tight')
if cleanup:
    os.remove(input_file)

