local_search_mode=ARGS[1]
param=parse(Int64,ARGS[2])
println("USING LOCAL SEARCH MODE "*local_search_mode)
println("USING LOCAL SEARCH PARAM "*string(param))
include("vns.jl")
include("writing_table.jl")
include("utils.jl")

output_file="data_files//neighborhoods_"*local_search_mode*"_param_"*string(param)*"_results_table.tex"

folder="MD algorithm datasets"

prefixes=[]
files=readdir(folder)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end


# prefixes=["MM16"]
#prefixes=["MM6"]
push!(prefixes,"MM1") # as warmup to get rid of load time discrepency
prefixes=sort(prefixes,by=number_from_file)
f=open(output_file,"w")
write_table_head(f)
folder='\\'*folder
warmup=true
for file in prefixes
    global warmup
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
    solver=Solver(instance;
                    debug=false,
                    mode_local_search = local_search_mode,
                    local_search_param_dicts = 
                        [Dict("num_vhcls_switch" => param, "vhcl_removal_choices" => ["longest tour","most targets"]) for _ in 1:length(local_search_mode)])
    println("DONE SOLVING")
    println("\tSTART TIME:",solver.start_time)
    println("\tPROGRESS:",sort(solver.progress))
    println("\tINITIAL OBJ:\t",solver.initial_solution.soln_cost)
    println("\tLOCAL SRCH OBJ:\t",solver.local_search_solution.soln_cost)
    println("\tFINAL OBJ:\t",solver.final_solution.soln_cost)
    println("\tINITIAL time:\t",solver.initial_time)
    println("\tLOCAL SRCH time:\t",solver.local_search_time)
    println("\tFINAL time:\t",solver.perturbation_time)
    route=solver.final_solution
    if !check_if_valid(route)
        println("ERROR: INVALID SOLUTION")
        exit()
    end
    dwell_times=Float64[]
    for vhcl_no in 1:instance.dim_depots
        tour=route.vhcl_tours[vhcl_no]
        if length(tour)<3
            push!(dwell_times,0)
        else
            targets=tour[2:length(tour)-1]
            avg=sum([route.vhcl_dwell_times[vhcl_no][t] for t in targets])/length(targets)
            #println([route.vhcl_dwell_times[vhcl_no][t] for t in targets])
            push!(dwell_times,avg)
        end
    end
    comp_times=[solver.initial_time,solver.local_search_time,solver.perturbation_time]
    #println(dwell_times)

    #string formatting
    #dwell_times=string(dwell_times)
    #dwell_times=dwell_times[2:length(dwell_times)-1]
    #println("\tFINAL SOLN:",solver.final_solution)
    # now just put them in files 
    if !warmup
        write_table_entry(f,file,alpha,tau,
                                [solver.initial_solution.soln_cost,[solver.initial_solution.vhcl_tour_costs[k] for k in 1:instance.dim_depots]],
                                [solver.local_search_solution.soln_cost,[solver.local_search_solution.vhcl_tour_costs[k] for k in 1:instance.dim_depots]],
                                [solver.final_solution.soln_cost,[solver.final_solution.vhcl_tour_costs[k] for k in 1:instance.dim_depots]],
                        dwell_times,comp_times)
    end
    warmup=false
end

write_table_tail(f,local_search_mode,param)
close(f)