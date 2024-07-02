import ast,numpy as np, os
from matplotlib import pyplot as plt

plot_dir=os.path.join("output","plots","proxy_cost_comparison")
if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)
input_dir=os.path.join("output","data_files","proxy_cost_comparison")
for mode in 'removal','insertion':
    filename=mode+"_data.txt"
    file=os.path.join(input_dir,filename)
    f=open(file,'r')
    dic=ast.literal_eval(f.read())
    f.close()
    overall=[]
    diffs_by_instance=dict()
    for instance in dic:
        print(instance)
        data=dic[instance]
        #data.sort()
        overall+=data
        data=np.array(data)

        indices=np.where(np.sign(data[:,0]).flatten()!=np.sign(data[:,1]).flatten())[0]

        # plot scatterplot
        plt.scatter(data[:,0],data[:,1],label="proxy costs",s=10)
        plt.scatter(data[indices,0],data[indices,1],s=2,color='red')

        plt.plot(data[:,0],data[:,0],'--',color='green',alpha=.5,label='y=x')
        plt.legend()
        plt.ylabel("Predicted Objective Change")
        plt.xlabel("True Objective Change")
        plt.savefig(os.path.join(plot_dir,instance+"_"+mode+"_proxy_scatter.png"))
        plt.close()

        # plot by target        
        plt.scatter(range(len(data)),data[:,1],label="Proxy estimate",color='blue',marker='x')
        plt.scatter(range(len(data)),data[:,0],label="Real objective change",color='red',marker='+')
        plt.legend()
        plt.xlabel("Target Number")
        plt.ylabel("Vehicle Objective Change")
        plt.savefig(os.path.join(plot_dir,instance+"_"+mode+"_proxy_scatter_by_target.png"))
        plt.close()
        diffs=data[:,1]-data[:,0]
        diffs_by_instance[instance]=diffs
        #print(diffs)
        print(max(diffs))

    # now plot all of them 

    #overall.sort()
    data=np.array(overall)

    indices=np.where(np.sign(data[:,0]).flatten()!=np.sign(data[:,1]).flatten())[0]

    # scatter plot
    plt.scatter(data[:,0],data[:,1],label="proxy costs",s=10)
    plt.scatter(data[indices,0],data[indices,1],s=2,color='red')

    plt.plot(data[:,0],data[:,0],'--',color='green',alpha=.5,label='y=x')
    plt.legend()
    plt.ylabel("Predicted Objective Change")
    plt.xlabel("True Objective Change")
    plt.savefig(os.path.join(plot_dir,"overall_"+mode+"_proxy_scatter.png"))
    plt.close()

    #plt.scatter(range(len(data)),data[:,1],label="proxy costs",color='blue',marker='x')
    #plt.scatter(range(len(data)),data[:,0],label="real costs",color='red',marker='+')
    #plt.legend()
    #plt.xlabel("Target Number")
    #plt.ylabel("Objective")
    #plt.savefig(os.path.join(plot_dir,"overall_"+mode+"_proxy_scatter_by_target.png"))
    #plt.close()
    print(mode)
    diffs=data[:,1]-data[:,0]
    print("max:",np.max(diffs))
    print("mean:",np.mean(diffs))
    print("median:",np.median(diffs))
    print("min:",np.min(diffs))
    print("pos:",len(np.where(diffs>0)[0]),"out of",len(diffs))

    # box plot
    all_data=[]
    labels=["MM"+str(i) for i in range(1,44)]
    for instance in labels:
        print(instance)
        if instance in dic:
            data=dic[instance]
            #data.sort()
            data=np.array(data)


            diffs=data[:,1]-data[:,0]
            if max(diffs)>0:
                print(max(diffs))
            all_data.append(diffs)


    plt.boxplot(all_data, vert=True, patch_artist=False, #labels=labels
                 )
    count=10
    step=(len(all_data)-1)/(count-1)
    indices=[int(round(i*step))+1 for i in range(count)]
    plt.xticks(indices,["MM"+str(i) for i in indices])
    
    #plt.tick_params( bottom=False,labelbottom=False)

    #plt.setp( ax.get_xticklabels(), visible=False)
    plt.title("Real objective change - Proxy estimate")
    plt.ylabel("Difference")
    plt.xlabel("Instance")
    plt.savefig(os.path.join(plot_dir,"overall_"+mode+"_proxy_boxy.png"))
    plt.close()

    print()