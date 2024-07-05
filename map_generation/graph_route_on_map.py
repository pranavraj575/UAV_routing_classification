from matplotlib import pyplot as plt, image as um
import os,sys,ast 
import numpy as np
from src.route_graphing_utils import graph_route

DIR=os.path.dirname(os.path.dirname(os.path.join(os.getcwd(),sys.argv[0])))
data_file=os.path.join(DIR,"output","data_files","experiment_results","real_world_all_values.txt")
dic=ast.literal_eval(open(data_file,'r').read())


alpha_tau=(1,1)
net_params=("12",2)
color_list=['purple','cyan','magenta','yellow','blue','orange',]

for key,dist in (("Berlin_0_1024",511.6),("NewYork_0_1024",511.8)):
    image_file=os.path.join(DIR,"output","plots","generated_maps",key+".png")
    
    depots=np.loadtxt(os.path.join("output","data_files","generated_maps",key+"_depots.txt"))
    targets=np.loadtxt(os.path.join("output","data_files","generated_maps",key+"_targets.txt"))

    this_dic=dic[alpha_tau][net_params][key]
    tours=this_dic['final_solution'][0]

    im=um.imread(image_file)
    plt.imshow(im,extent=[-dist/2,dist/2,-dist/2,dist/2])

    graph_route(targets=targets,
                depots=depots,
                tours=tours,
                tour_labels=None,
                labels=None,
                color_list=color_list,
                rotation=np.array([[0,1],[-1,0]]),
                depot_color="white",
                target_color="red",
                node_size=7,
                edge_width=1.5,
                random_seed=10,
                ax=plt.gca(),
                )

    plt.savefig(os.path.join("output","plots","generated_maps",key+"_route_output.png"),bbox_inches='tight')
    plt.close()