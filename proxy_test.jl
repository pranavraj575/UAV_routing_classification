include("construct.jl")
include("utils.jl")
include("neighborhoods.jl")

num_vhcls_switch=1
tol=.000001

data_file_removal="data_files/proxy_cmp_removal.txt"
data_file_insertion="data_files/proxy_cmp_insertion.txt"

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
# prefixes=["MM3"]
prefixes=sort(prefixes,by=number_from_file)

removal_pairs_by_instance=Dict()
insertion_pairs_by_instance=Dict()
folder='\\'*folder
for file in prefixes
    removal_pairs_by_instance[file]=[]
    insertion_pairs_by_instance[file]=[]
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
    route=partition_tours_star_heuristic(instance)
    #vhcl_no_remove=choose_vhcls_for_removal(route,["longest tour"],1)[1]
    for vhcl_no_remove in 1:route.instance.dim_depots

        removal_original_objective=route.vhcl_objective_func_costs[vhcl_no_remove]

        diffs_target_sorted = removal_sorted_vhcl_i(route, vhcl_no_remove)
        for ((t, index, diff_tour), diff_obj) in diffs_target_sorted
            route_remove_targ=route_copy(route)

            deleteat!(route_remove_targ.vhcl_tours[vhcl_no_remove], index)

            dwell_time_t = route_remove_targ.vhcl_dwell_times[vhcl_no_remove][t]
            route_remove_targ.vhcl_dwell_times[vhcl_no_remove][t] = 0

            new_tour_cost_vhcl = route_remove_targ.vhcl_tour_costs[vhcl_no_remove] + diff_tour
            new_obj_val_vhcl = route_remove_targ.vhcl_objective_func_costs[vhcl_no_remove] + diff_obj
            route_remove_targ =
            route_set_vhcl_i_tour_cost(route_remove_targ, new_tour_cost_vhcl, new_obj_val_vhcl, vhcl_no_remove)

            removal_test=route_copy(route_remove_targ)
            removal_test=update_dwell_times_tours_costs_obj(removal_test,vhcl_no_remove,false,tolerance=tol)
            real_vhcl_cost=removal_test.vhcl_objective_func_costs[vhcl_no_remove]

            real_diff=real_vhcl_cost-removal_original_objective
            push!(removal_pairs_by_instance[file],(real_diff,diff_obj))
            #println("guessed diff: ",diff_obj,"; real diff: ",real_diff)
            #println("guessed val:",new_obj_val_vhcl,", updated: ",real_vhcl_cost)
            if diff_obj>real_diff
                println()
                println("guessed tour cost: ",new_tour_cost_vhcl,"; real tc: ",removal_test.vhcl_tour_costs[vhcl_no_remove])
                println("tour cost improvement: ",-removal_test.vhcl_tour_costs[vhcl_no_remove]+new_tour_cost_vhcl)
                println("guessed removal diff: ",diff_obj,"; real diff: ",real_diff)
                println("improvement: ",real_diff-diff_obj)
                println("guessed removal val:",new_obj_val_vhcl,", updated: ",real_vhcl_cost)

            end
            # insertion
            vhcls_pos_insertion = vhcls_sorted_insertion_diff(route, vhcl_no_remove, t)

            for i in 1:num_vhcls_switch
                ((vhcl_no_insertion, insertion_diff_tour, pos_insertion), insertion_diff_obj) =
                vhcls_pos_insertion[i]

                route_remove_insert_t = route_copy(route_remove_targ)
                insertion_initial_objective=route_remove_insert_t.vhcl_objective_func_costs[vhcl_no_insertion]

                insert!(route_remove_insert_t.vhcl_tours[vhcl_no_insertion], pos_insertion, t)
                route_remove_insert_t.vhcl_dwell_times[vhcl_no_insertion][t] = dwell_time_t
                new_tour_cost_vhcl = route_remove_insert_t.vhcl_tour_costs[vhcl_no_insertion] + insertion_diff_tour
                new_obj_val_vhcl = route_remove_targ.vhcl_objective_func_costs[vhcl_no_insertion] + insertion_diff_obj
                route_remove_insert_t =
                route_set_vhcl_i_tour_cost(route_remove_insert_t, new_tour_cost_vhcl, new_obj_val_vhcl, vhcl_no_insertion)

                insertion_test=route_copy(route_remove_insert_t)
                insertion_test=update_dwell_times_tours_costs_obj(insertion_test,vhcl_no_insertion,false,tolerance=tol)
                real_vhcl_cost=insertion_test.vhcl_objective_func_costs[vhcl_no_insertion]
                real_insert_diff=real_vhcl_cost-insertion_initial_objective
                
                push!(insertion_pairs_by_instance[file],(real_insert_diff,insertion_diff_obj))
                if insertion_diff_obj>real_insert_diff
                    println()
                    println("guessed tour cost: ",new_tour_cost_vhcl,"; real tc: ",insertion_test.vhcl_tour_costs[vhcl_no_insertion])
                    println("tour cost improvement: ",-insertion_test.vhcl_tour_costs[vhcl_no_insertion]+new_tour_cost_vhcl)
                    println("guessed insert diff: ",insertion_diff_obj,"; real diff: ",real_insert_diff)
                    println("improvement: ",real_insert_diff-insertion_diff_obj)
                    println("guessed insert val:",new_obj_val_vhcl,", updated: ",real_vhcl_cost)

                end
            end
        end
    end
end

for (data_file,dic) in ((data_file_removal,removal_pairs_by_instance),(data_file_insertion,insertion_pairs_by_instance))
    f=open(data_file,"w")
    write(f,"{\n")
    for key in keys(dic)
        write(f,"\t\""*string(key)*"\":[\n")
        for (real,fake) in dic[key]
            write(f,"\t\t(")
            write(f,string(real))
            write(f,", ")
            write(f,string(fake))
            write(f,"),\n")
        end
        write(f,"\t],\n")
    end
    write(f,"}")
    close(f)
end
