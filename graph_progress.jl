include(joinpath("src","instance.jl"))
include(joinpath("src","vns.jl"))
include(joinpath("src","network_graphing.jl"))
include(joinpath("src","utils.jl"))
param=2
local_search_mode="12"

plot_dir=joinpath("output","route_plots")

output_file_start="neighborhoods_"*local_search_mode*"_param_"*string(param)
if !isdir(plot_dir)
    mkpath(plot_dir)
end

folder=joinpath("input_data", "MD_algorithm_datasets")

prefixes=[]
files=readdir(folder)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end
# prefixes=["MM12"]

prefixes=sort(prefixes, by=number_from_file)

for file in prefixes
    println("\n\n",file,"\t",folder)
    data=Data(file,folder)
    tau=2.
    # figuring out alpha
    test_alpha=1.
    # alpha is 1/tsp cost
    test_instance=Instance(data,test_alpha,tau)
    test_tour=construct_tour_LKH(test_instance,collect(keys(test_instance.targets)))
    test_tour_cost=path_cost(test_instance,test_tour)    
    println("TEST TOUR COST: ",test_tour_cost)
    alpha=1/test_tour_cost
    println("ALPHA: ",alpha)

    instance=Instance(data,alpha,tau)
    #println("TEST: ",instance.cost_traversal[][1])
    solver=Solver(instance;mode_local_search = local_search_mode,local_search_param_dicts = 
                        [Dict("num_vhcls_switch" => param, "vhcl_removal_choices" => ["most targets"]) for _ in 1:length(local_search_mode)])

    if !check_if_valid(solver.final_solution)
        println("ERROR: INVALID SOLUTION")
        exit()
    end
    for (name,route) in (("initial",solver.initial_solution),("local_search",solver.local_search_solution),("final",solver.final_solution),)
        output_file_name=joinpath(plot_dir,output_file_start*"_instance_"*file*"_"*name*"_solution.png")
        graph_route(route,output_file_name)
        println(name)
        println("OBJ: ",route.soln_cost)
        println("TOUR COSTS: ",route.vhcl_tour_costs)
    end
end
