import ast,numpy as np
from matplotlib import pyplot as plt
for mode in 'removal','insertion':
    file="data_files/proxy_cmp_"+mode+".txt"
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
        plt.savefig("proxy_cmp_plots\\"+instance+"_"+mode+"_proxy_scatter.png")
        plt.close()

        # plot by target        
        plt.scatter(range(len(data)),data[:,1],label="Proxy estimate",color='blue',marker='x')
        plt.scatter(range(len(data)),data[:,0],label="Real objective change",color='red',marker='+')
        plt.legend()
        plt.xlabel("Target Number")
        plt.ylabel("Vehicle Objective Change")
        plt.savefig("proxy_cmp_plots\\"+instance+"_"+mode+"_proxy_scatter_by_target.png")
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
    plt.savefig("proxy_cmp_plots\\overall_"+mode+"_proxy_scatter.png")
    plt.close()

    #plt.scatter(range(len(data)),data[:,1],label="proxy costs",color='blue',marker='x')
    #plt.scatter(range(len(data)),data[:,0],label="real costs",color='red',marker='+')
    #plt.legend()
    #plt.xlabel("Target Number")
    #plt.ylabel("Objective")
    #plt.savefig("proxy_cmp_plots\\overall_"+mode+"_proxy_scatter_by_target.png")
    #plt.close()
    print(mode)
    diffs=data[:,1]-data[:,0]
    print("max:",np.max(diffs))
    print("mean:",np.mean(diffs))
    print("median:",np.median(diffs))
    print("min:",np.min(diffs))
    print("pos:",len(np.where(diffs>0)[0]),"out of",len(diffs))
    print()