include(joinpath("src","vns.jl"))
include(joinpath("src","utils.jl"))
include("writing_table.jl")
data_file=joinpath("temp","new_large_plot_alpha_2_tau_1.txt")
folder="MD algorithm datasets"
trials=1

prefixes=[]
files=readdir(folder)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end
# prefixes=["MM16"]
# prefixes=[ "MM"*string(num) for num in [18]]
prefixes=sort(prefixes,by=number_from_file)
warmup=true
dic=Dict()
println("SAVING TO: ",data_file)
for alpha_factor in [2]
    for tau in [1.]
        global dic
        dic[(alpha_factor,tau)]=Dict()
        println("USING ALPHA FACTOR: ",alpha_factor)
        println("USING TAU: ",tau)
        for local_search_mode in ["1","12"]
            for param in [1,2]
                global dic
                dic[(alpha_factor,tau)][(local_search_mode,param)]= Dict(fix=>Dict(
                                                                            "compute_times"=>Float64[],
                                                                            "percent_improvements"=>Float64[],
                                                                            "initial_obj"=>Float64[],
                                                                            "initial_time"=>Float64[],
                                                                            "local_search_obj"=>Float64[],
                                                                            "local_search_time"=>Float64[],
                                                                            "final_obj"=>Float64[],
                                                                            ) 
                                                                        for fix in prefixes)
                
                println("\tUSING LOCAL SEARCH MODE "*local_search_mode)
                println("\tUSING LOCAL SEARCH PARAM "*string(param))
                for trial in 1:trials
                    println("\t\tTRIAL: ",trial)
                    global warmup
                    loc_prefixes=copy(prefixes)
                    if warmup
                        insert!(loc_prefixes,1,"MM1")
                    end

                    for file in loc_prefixes
                        global warmup
                        println("\t\t\t",file,"\t",folder)
                        data=Data(file,'\\'*folder)
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
                        final_time=solver.perturbation_time
                        percent_improvement=100*(final_obj-initial_obj)/initial_obj
                        if !warmup
                            push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["compute_times"],
                                        final_time)
                            push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["percent_improvements"],
                                        percent_improvement)
                            

                            
                            push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["initial_obj"],
                                        initial_obj)    
                            push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["initial_time"],
                                        solver.initial_time)
                            push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["local_search_obj"],
                                        solver.local_search_solution.soln_cost)  
                            push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["local_search_time"],
                                        solver.local_search_time)
                            push!(dic[(alpha_factor,tau)][(local_search_mode,param)][file]["final_obj"],
                                        final_obj)

                            println("\t\t\t\tINITIAL OBJ:\t",initial_obj)
                            println("\t\t\t\tFINAL OBJ:\t",final_obj)
                            println("\t\t\t\tFINAL time:\t",final_time)
                        else
                            println("\t\t\t\tWARM UP DONE")
                        end
                        warmup=false
                    end
                end
            end
        end
    end
end
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