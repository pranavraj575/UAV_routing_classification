import ast,numpy as np
from matplotlib import pyplot as plt
for mode in 'insertion','removal':
    file="data_files/proxy_cmp_"+mode+".txt"
    f=open(file,'r')
    dic=ast.literal_eval(f.read())
    f.close()
    diffs_by_instance=dict()
    all_data=[]
    labels=["MM"+str(i) for i in range(1,44)]
    for instance in labels:
        print(instance)
        data=dic[instance]
        #data.sort()
        data=np.array(data)


        diffs=data[:,1]-data[:,0]
        diffs_by_instance[instance]=diffs
        #print(diffs)
        if max(diffs)>0:
            print(max(diffs))
        all_data.append(diffs)



    print(mode)
    diffs=data[:,1]-data[:,0]
    print("max:",np.max(diffs))
    print("mean:",np.mean(diffs))
    print("median:",np.median(diffs))
    print("min:",np.min(diffs))
    print("pos:",len(np.where(diffs>0)[0]),"out of",len(diffs))
    plt.boxplot(all_data, vert=True, patch_artist=False, #labels=labels
                 )
    count=10
    step=(len(all_data)-1)/(count-1)
    indices=[int(round(i*step))+1 for i in range(count)]
    plt.xticks(indices,["MM"+str(i) for i in indices])
    
    #plt.boxplot(diffs)
    #plt.tick_params( bottom=False,labelbottom=False)

    #plt.setp( ax.get_xticklabels(), visible=False)
    plt.title("Real objective change - Proxy estimate")
    plt.ylabel("Difference")
    plt.xlabel("Instance")
    plt.savefig("proxy_cmp_plots\\overall_"+mode+"_proxy_boxy.png")
    plt.close()

    print()