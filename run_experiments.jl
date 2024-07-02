# creates dataset comparing hyperparameters across all MD algorithm datasets
# hyperparams are 
#     1pt move neighborhoods or both 1pt move and swap neighborhoods
#     top choice or top two choices
# plots can be created with neighborhood_type_comparison.py
# output can be compared to memetic in memetic_plot.py


include(joinpath("src","vns.jl"))
include(joinpath("src","utils.jl"))
include(joinpath("src","network_graphing.jl"))

# number of times to run each experiment
trials=3
# these are the parameters that we will run
# note that we check the cartesian product of all of these
alpha_factors=[1]#,2]
tau_values=[1.]#,2.]
local_search_modes=["1","12"]
param_choices=[1,2]


save_routes=true

if true
    #experiment_name="MD_all_values"
    experiment_name="MD_alpha_1_tau_1"
    input_dir=joinpath("input_data","MD_algorithm_datasets")
else
    experiment_name="real_world_all_values"
    input_dir=joinpath("output","data_files","generated_maps")
end

data_dir=joinpath("output","data_files","experiment_results")
plot_dir=joinpath("output","plots","compare_to_optimal")


data_file=joinpath(data_dir,experiment_name*".txt")
for d in (data_dir,plot_dir)
    if !isdir(d)
        mkpath(d)
    end
end
prefixes=[]
files=readdir(input_dir)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end
# prefixes=["MM1"]
# prefixes=[ "MM"*string(num) for num in [18]]
prefixes=sort(prefixes,by=number_from_file)
warmup=true
dic=Dict()
println("SAVING TO: ",data_file)
for alpha_factor in alpha_factors
    for tau in tau_values
        global dic
        dic[(alpha_factor,tau)]=Dict()
        println("USING ALPHA FACTOR: ",alpha_factor)
        println("USING TAU: ",tau)
        for local_search_mode in local_search_modes
            for param in param_choices
                global dic
                temp_dic= Dict(prefix=>Dict() for prefix in prefixes)
                
                println("\tUSING LOCAL SEARCH MODE "*local_search_mode)
                println("\tUSING LOCAL SEARCH PARAM "*string(param))
                for trial in 1:trials
                    println("\t\tTRIAL: ",trial)
                    global warmup
                    loc_prefixes=copy(prefixes)
                    if warmup
                        println("WARMING UP WITH ",prefixes[1])
                        insert!(loc_prefixes,1,prefixes[1])
                    end

                    for file in loc_prefixes
                        global warmup
                        println("\t\t\t",file,"\t",input_dir)
                        data=Data(file,input_dir)
                        # figuring out alpha
                        # alpha is 1/tsp cost
                        test_instance=Instance(data,1.,tau)
                        test_tour=construct_tour_LKH(test_instance,collect(keys(test_instance.targets)))
                        test_tour_cost=path_cost(test_instance,test_tour)    
                        alpha=alpha_factor/test_tour_cost

                        instance=Instance(data,alpha,tau)
                        solver=Solver(instance;
                                        debug=false,
                                        mode_local_search = local_search_mode,
                                        local_search_param_dicts = 
                                            [Dict("num_vhcls_switch" => param, 
                                                "vhcl_removal_choices" => ["longest tour","most targets"]) for _ in 1:length(local_search_mode)])
                        if !check_if_valid(solver.final_solution)
                            println("ERROR: INVALID SOLUTION")
                            exit()
                        end
                        initial_obj=solver.initial_solution.soln_cost
                        final_obj=solver.final_solution.soln_cost
                        percent_improvement=100*(final_obj-initial_obj)/initial_obj
                        if !warmup
                            solver_dic=solver_to_dict(solver)
                            solver_dic["percent_improvement"]=percent_improvement

                            if !save_routes
                                for k in ("initial_solution","local_search_solution","final_solution")
                                    delete!(solver_dic,k)
                                end
                            end

                            for key in keys(solver_dic)
                                if key in keys(temp_dic[file])
                                    push!(temp_dic[file][key], solver_dic[key])
                                else
                                    temp_dic[file][key]=[solver_dic[key]]
                                end
                            end
                            println("\t\t\t\tINITIAL OBJ:\t",initial_obj)
                            println("\t\t\t\tFINAL OBJ:\t",final_obj)
                            println("\t\t\t\tFINAL time:\t",solver.perturbation_time)
                            identifier="neighborhoods_"*local_search_mode*"_param_"*string(param)*"_alpha_"*string(alpha_factor)*"_tau_"*string(tau)
                            plotname=joinpath(plot_dir,file*identifier*"_ours.png")
                            graph_route(solver.final_solution,plotname)
                        else
                            println("\t\t\t\tWARM UP DONE")
                        end
                        warmup=false
                    end
                end
                dic[(alpha_factor,tau)][(local_search_mode,param)]=temp_dic
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