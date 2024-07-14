from matplotlib import pyplot as plt, image as um
import os,sys,ast 
import numpy as np
from src.route_graphing_utils import graph_route

DIR=os.path.dirname(os.path.dirname(os.path.join(os.getcwd(),sys.argv[0])))
data_file=os.path.join(DIR,"output","data_files","experiment_results","real_world_all_values.txt")
dic=ast.literal_eval(open(data_file,'r').read())


alpha_tau=(1,1)
net_params=("12",2)
color_list=['#000000',
            '#00ffff',
            '#0000ff',
            '#ff00ff',
            'orange',
            'purple',
            '#00ff00',

            '#007f7f',
            '#00007f',
            '#7f007f',
            '#007f00',
            ]

keys_dists=[(key,511.6) for key in dic[alpha_tau][net_params].keys()]

for key,dist in keys_dists:
    image_file=os.path.join(DIR,"output","plots","generated_maps",key+".png")
    
    depots=np.loadtxt(os.path.join("output","data_files","generated_maps",key+"_depots.txt"))
    targets=np.loadtxt(os.path.join("output","data_files","generated_maps",key+"_targets.txt"))

    this_dic=dic[alpha_tau][net_params][key]
    print('doing',key)
    print('alpha value:',this_dic['alpha_value'])
    print('tau value:',alpha_tau[1])
    print()
    tours=this_dic['final_solution'][0]
    dwell_times=this_dic['final_dwell_times'][0]
    avg_dwell_times=[np.mean(t) for t in dwell_times]

    labels=[
        #"vhcl "+str(tour_no)+": 
        "$\\overline{d_i}$="+str(round(d_t,2))
        for tour_no,d_t in enumerate(avg_dwell_times)
    ]

    im=um.imread(image_file)
    plt.imshow(im,extent=[-dist/2,dist/2,-dist/2,dist/2])

    graph_route(targets=targets,
                depots=depots,
                tours=tours,
                tour_labels=labels,
                with_labels=False,
                color_list=color_list,
                rotation=np.array([[0,1],[-1,0]]),
                depot_color="#0000ff",
                target_color="#ff0000",
                node_size=7,
                edge_width=1.5,
                random_seed=10,
                ax=plt.gca(),
                )
    ncol=np.ceil(np.sqrt(len(depots)))
    legend_height=1+.07*(np.ceil(len(depots)/ncol))
    plt.legend(loc='upper center', bbox_to_anchor=(.5, legend_height),ncol=ncol)
    plt.axis('off') 
    plt.rc('font', size=10) 
    plt.savefig(os.path.join("output","plots","generated_maps",key+"_route_output.png"),bbox_inches='tight')
    plt.close()