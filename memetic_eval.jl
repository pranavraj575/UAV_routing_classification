include(joinpath("src","vns.jl"))
include(joinpath("src","utils.jl"))
include(joinpath("src","network_graphing.jl"))

param=2
local_search_mode="12"
alpha_factor=1
tau=2

identifier="new_neighborhoods_"*local_search_mode*"_param_"*string(param)*"_alpha_"*string(alpha_factor)*"_tau_"*string(tau)

println(identifier)

output_file=joinpath("data_files","memetic_comparision_"*identifier*".txt")
plot_dir="compare_to_optimal"

test_file=joinpath("temp","memetic_cmp_meme_data_"*identifier*".txt")

function extract_tour(line)
    if '.' in line || 't' in line  || line == ""
        return []
    end
    return [parse(Int64, v) for v in split(line,' ')]
end

folder="MD algorithm datasets"
other_folder="filesformemeticalgorithm"
prefixes=[]
files=readdir(folder)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end
#prefixes=["MM10"]
prefixes=sort(prefixes,by=number_from_file)
#f=open(output_file,"w")
g=open(test_file,"w")
#write(f,"{\n")
write(g,"{\n")
for file in prefixes
    other_file=replace(file,"MM"=>"md")*".txt"
    if !(other_file in readdir(other_folder))
        continue
    end
    #write(f,"'"*file*"'")
    #write(f,":")
    write(g,"'"*file*"'")
    write(g,":")
    println("\n\n",file,"\t",folder)
    text=prod([string(Char(chr)) for chr in read(open(joinpath(other_folder,other_file)))])
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
    


    data=Data(file,folder)
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
    #solver=Solver(instance;
    #    mode_local_search = local_search_mode,
    #    local_search_param_dicts = 
    #    [Dict("num_vhcls_switch" => param, "vhcl_removal_choices" => ["longest tour","most targets"]) for _ in 1:length(local_search_mode)])
    #route2=solver.final_solution
    #println("OUR OBJECTIVE: ",route2.soln_cost)
    plotname=joinpath(plot_dir,file)
    graph_route(route1,plotname*"_memetic.png")
    #graph_route(route2,plotname*"ours_"*identifier*".png")

    #write(f,"{'memetic':")
    #write(f,string(route1.soln_cost))
    #write(f,",")
    #write(f,"'ours':")
    #write(f,string(route2.soln_cost))
    #write(f,"}")

    write(g,string(route1.soln_cost))
    write(g,",\n")
    
    #write(f,",\n")
end
#write(f,"\n}")
#close(f)
write(g,"\n}")
close(g)