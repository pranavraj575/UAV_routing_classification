# uses memetic tour output (MD dataset)
# uses these tours as tour solution, runs gradient descent, then dumps objective values into a file
# run memetic_plot.py to plot results

include(joinpath("src","vns.jl"))
include(joinpath("src","utils.jl"))
include(joinpath("src","network_graphing.jl"))


# parameters
# for calculating alpha and tau 
alpha_factors=[1,2]
tau_values=[2.,1.]
# which types of neighborhood to use in local search
local_search_modes=["12","1"]
# check top k results in local search
param_choices=[2,1]

plot_dir=joinpath("output","plots","compare_to_optimal")
data_dir=joinpath("output","data_files","experiment_results")

data_file=joinpath(data_dir,"testmemetic_results.txt")

for d in (plot_dir,data_dir)
    if !isdir(d)
        mkpath(d)
    end
end

function extract_tour(line)
    if '.' in line || 't' in line  || line == ""
        return []
    end
    return [parse(Int64, v) for v in split(line,' ')]
end

md_file_directory=joinpath("input_data","MD_algorithm_datasets")
memetic_file_directory=joinpath("input_data","filesformemeticalgorithm")
prefixes=[]
files=readdir(md_file_directory)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end
prefixes=["MM1"]

prefixes=sort(prefixes,by=number_from_file)

dic=Dict()
for alpha_factor in alpha_factors
    for tau in tau_values
        global dic
        dic[(alpha_factor,tau)]=Dict()
        println("USING ALPHA FACTOR: ",alpha_factor)
        println("USING TAU: ",tau)
        for local_search_mode in local_search_modes
            for param in param_choices
                global dic
                dic[(alpha_factor,tau)][(local_search_mode,param)]= Dict(fix=>Dict(
                                                                            "final_obj"=>Float64[],
                                                                            ) 
                                                                        for fix in prefixes)
                
                println("\tUSING LOCAL SEARCH MODE "*local_search_mode)
                println("\tUSING LOCAL SEARCH PARAM "*string(param))

                
                for file in prefixes
                    other_file=replace(file,"MM"=>"md")*".txt"
                    if !(other_file in readdir(memetic_file_directory))
                        continue
                    end
                    println("\n\n",file,"\t",md_file_directory)
                    text=prod([string(Char(chr)) for chr in read(open(joinpath(memetic_file_directory,other_file)))])
                    tours=[]
                    depot_offset=0
                    max_target=0
                    for line in split(text,"\n")
                        result=[]
                        if strip(line)!=""
                            result=extract_tour(line)
                        end
                        if result==[]
                            continue
                        end
                        push!(tours,result)
                        depot_offset=max(depot_offset,result[1])
                        max_target=max(max_target,maximum(result))
                    end
                    # shift everything down by depot_offset, move the depots to max_target+k
                    max_target-=depot_offset
                    for (k,tour) in enumerate(tours)
                        for i in 2:length(tour)-1
                            tour[i]-=depot_offset
                        end
                        tour[1]=k+max_target
                        tour[length(tour)]=k+max_target
                    end

                    data=Data(file,md_file_directory)
                    # figuring out alpha
                    test_alpha=1.
                    # alpha is 1/tsp cost
                    test_instance=Instance(data,test_alpha,tau)
                    test_tour=construct_tour_LKH(test_instance,collect(keys(test_instance.targets)))
                    test_tour_cost=path_cost(test_instance,test_tour)    
                    println("TEST TOUR COST: ",test_tour_cost)
                    alpha=alpha_factor/test_tour_cost
                    println("ALPHA: ",alpha)

                    instance=Instance(data,alpha,tau)
                    route1=Route(instance)
                    
                    for (vhcl_no,tour) in enumerate(tours)
                        depot_key=collect(keys(instance.depots))[vhcl_no]
                        vertices=tour[2:length(tour)-1]
                        # correct number of taus
                        tau_arr=[instance.tau for _ in vertices]
                        dwell_times=nesterov_gradient_descent(tau_arr,instance.alpha,1.)
                        tour_cost=path_cost(instance,tour)
                        G=undiscounted_info_gain_func(dwell_times,tau_arr)
                        objective_value=exp(-instance.alpha*tour_cost)*exp(-instance.alpha*sum(dwell_times))*G
                        route_set_vhcl_i_tour_tour_cost(route1,tour,tour_cost,objective_value,vhcl_no)
                        for (i,key) in enumerate(vertices)
                            route1.vhcl_dwell_times[vhcl_no][key]=dwell_times[i]
                        end
                    end
                    println("MEMETIC OBJECTIVE: ",route1.soln_cost)
                    identifier="neighborhoods_"*local_search_mode*"_param_"*string(param)*"_alpha_"*string(alpha_factor)*"_tau_"*string(tau)

                    plotname=joinpath(plot_dir,file*identifier*"_memetic.png")
                    graph_route(route1,plotname)
                    push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["final_obj"],
                        route1.soln_cost)
                end
            end
        end
    end
end

# write results into dictionary format
f=open(data_file,"w")
write(f,"{\n")
for key in keys(dic)
    write(f,"\t"*string(key)*":{\n")
    for key2 in keys(dic[key])
        write(f,"\t\t"*string(key2)*":{\n")
        for file in keys(dic[key][key2])
            write(f,"\t\t\t'"*string(file)*"':{\n")
            for key3 in keys(dic[key][key2][file])
                listt=dic[key][key2][file][key3]
                write(f,"\t\t\t\t'"*string(key3)*"':"*string(listt)*",\n")
            end
            write(f,"\t\t\t},\n")
        end
        write(f,"\t\t},\n")
    end
    write(f,"\t},\n")
end
write(f,"}")
close(f)
