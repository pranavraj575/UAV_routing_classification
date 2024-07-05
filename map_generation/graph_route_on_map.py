from matplotlib import pyplot as plt, image as um
import os,sys,ast 
import networkx as nx
import numpy as np
DIR=os.path.dirname(os.path.dirname(os.path.join(os.getcwd(),sys.argv[0])))
data_file=os.path.join(DIR,"output","data_files","experiment_results","real_world_all_values.txt")
dic=ast.literal_eval(open(data_file,'r').read())


alpha_tau=(1,1)
net_params=("12",2)

depot_color="white"
target_color="red"
edge_width=1.5
node_size=7
np.random.seed(10)

color_vectors=[np.array([float(i==k) for k in range(3)]) for i in range(3)]
def rand_color():
    v=np.array(color_vectors)@np.random.random(3)

    return v/np.linalg.norm(v)
color_list=['purple','cyan','magenta','yellow','blue','orange',]+[rand_color() for _ in range(100)]

rot=np.array([[0,1],[-1,0]])

for key,dist in (("Berlin_0_1024",511.6),("NewYork_0_1024",511.8)):
    image_file=os.path.join(DIR,"output","plots","generated_maps",key+".png")
    
    depots=np.loadtxt(os.path.join("output","data_files","generated_maps",key+"_depots.txt"))
    targets=np.loadtxt(os.path.join("output","data_files","generated_maps",key+"_targets.txt"))

    buildings=np.concatenate((targets,depots),axis=0)

    this_dic=dic[alpha_tau][net_params][key]
    tours=this_dic['final_solution'][0]
    
    im=um.imread(image_file)
    plt.imshow(im,extent=[-dist/2,dist/2,-dist/2,dist/2])
    G = nx.DiGraph()
    for i in range(len(targets)):
        G.add_node(i+1, pos=rot@buildings[i], color=target_color)
    
    for i in range(len(targets), len(buildings)):
        G.add_node(i+1, pos=rot@buildings[i], color=depot_color)
    for tour_no,tour in enumerate(tours):
        color=color_list[tour_no]
        for i in range(len(tour)-1):
            G.add_edge(tour[i],tour[i+1],color=color,width=edge_width)
    
    pos=(nx.get_node_attributes(G,"pos"))

    node_color = list(nx.get_node_attributes(G,'color').values())
    edge_color = list(nx.get_edge_attributes(G,'color').values())
    edge_widths = list(nx.get_edge_attributes(G,'width').values())
    nx.draw_networkx(G,
        pos=pos,
        with_labels=False, # whether to label targets
        font_weight='bold', 
        node_color=node_color,
        edge_color=edge_color,
        node_size=node_size,
        width=edge_widths,
        ax=plt.gca(),
        )
    
    plt.savefig(os.path.join("output","plots","generated_maps",key+"_route_output.png"),bbox_inches='tight')
    plt.close()