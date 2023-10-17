using OrderedCollections
include("route.jl")
include("instance.jl")
include("highs_single_vehicle_TSP_dwelltime_functions.jl")

function update_dwell_times_tours_costs_obj(route,vhcl_no,do_exact;tolerance=.0001)
    # updates the tour with lkh and the dwell times with gradient descent
    # vertices used by the insertion vehicle (not including depot)
    vertices=route.vhcl_tours[vhcl_no]
    vertices=vertices[2:length(vertices)-1]

    old_dwell_times=[route.vhcl_dwell_times[vhcl_no][v] for v in vertices]
    
    # solving for tour and dwell times
    (tour, dwell_times, cost, key_array)=single_vehicle_TSP_dwell_times(route.instance,
                                                                        vertices,
                                                                        vhcl_no,
                                                                        do_exact,initial=old_dwell_times,tolerance=tolerance)
    # updating
    route.vhcl_tours[vhcl_no]=tour
    reset_dwell_times_vhcl(route,vhcl_no)
    for (i,key) in enumerate(key_array)
        route.vhcl_dwell_times[vhcl_no][key]=dwell_times[i]
    end
    route.vhcl_tour_costs[vhcl_no]=cost
    old_vhcl_obj=route.vhcl_objective_func_costs[vhcl_no]
    G=undiscounted_info_gain_func(dwell_times,[route.instance.tau for _ in dwell_times])
    new_vhcl_obj=exp(-route.instance.alpha*(sum(dwell_times)+cost))*G

    route.vhcl_objective_func_costs[vhcl_no]=new_vhcl_obj
    route.soln_cost+=(new_vhcl_obj-old_vhcl_obj)

    return route
end

function choose_vhcls_for_removal(route,choice_types,number_per_type)
    # returns a list of vhcl numbers
    # which vehicles to remove a target from 
    # currently doing based on max number of targets and max objective value
    out=[]
    for arg in choice_types
        if arg=="most targets"
            fun=sorted_vhcls_most_targets
        elseif arg=="largest obj"
            fun=sorted_vhcls_largest_obj_val
        elseif arg=="smallest obj"
            fun=sorted_vhcls_smallest_obj_val
        elseif arg=="longest tour"
            fun=sorted_vhcls_longest_tour
        else
            println("ERROR: '",arg,"' not in options to choose from")
            exit()
        end
        sorted_list=fun(route)
        for i in 1:min(number_per_type,length(sorted_list))
            vhcl_no=sorted_list[i]
            if !(vhcl_no in out)
                push!(out,vhcl_no)
            end
        end
    end
    return out
end

function one_pt_move(route, param_dict)
    # In this function, a target switch from the vehicle with the least objective
    # value to a vehicles sorted in decreasing order of insertion difference is performed.
    if route.instance.dim_depots == 1
        # no vehicles to swap with
        return route
    end
    do_exact=false# whether to solve exactly or just use LKH

    num_vhcls_switch=param_dict["num_vhcls_switch"]
    vhcl_removal_choices=param_dict["vhcl_removal_choices"]

    # Obtaining vehicle with the least/most objective value
    for vhcl_no_remove in choose_vhcls_for_removal(route,vhcl_removal_choices,num_vhcls_switch)
        # Sorting targets in this vehicle in the decreasing order of difference
        diffs_target_sorted = removal_sorted_vhcl_i(route, vhcl_no_remove)
        # Running through each target in diffs_target_sorted and attempting to
        # remove targets with maximal potential diffs from the tour
        for ((t, index, diff_tour), diff_obj) in diffs_target_sorted
            # Making a copy of the object
            route_remove_targ = route_copy(route)
            # Removing target t from the vehicle with the least objective value, which
            # is located at "index" in the vehicle's tour
            deleteat!(route_remove_targ.vhcl_tours[vhcl_no_remove], index)
            # Checking if the vehicle with least objective value is not covering any
            # more targets. This occurs when the vehicle initially covered just one target.
            # Now, checking if the considered vehicle has a tour of length 2, which 
            # essentially means that the tour of the vehicle is from its depot back to the depot.
            if length(route_remove_targ.vhcl_tours[vhcl_no_remove]) == 2
                route_remove_targ.vhcl_tours[vhcl_no_remove] = []
            end

            # Updating the dwell time dictionary for the target removed from the considered
            # vehicle
            dwell_time_t = route_remove_targ.vhcl_dwell_times[vhcl_no_remove][t]
            route_remove_targ.vhcl_dwell_times[vhcl_no_remove][t] = 0
            # Updating the tour cost and objective value of considered vehicle
            new_tour_cost_vhcl = route_remove_targ.vhcl_tour_costs[vhcl_no_remove] + diff_tour
            new_obj_val_vhcl = route_remove_targ.vhcl_objective_func_costs[vhcl_no_remove] + diff_obj
            route_remove_targ =
            route_set_vhcl_i_tour_cost(route_remove_targ, new_tour_cost_vhcl, new_obj_val_vhcl, vhcl_no_remove)

            # Identifying the sorted list of vehicles in increasing order of insertion
            # cost metric
            vhcls_pos_insertion = vhcls_sorted_insertion_diff(route, vhcl_no_remove, t)

            # Running through the top num_vhcls_switch number of vehicles
            for i in 1:num_vhcls_switch
                # Obtaining the chosen vehicle index for insertion, position of insertion,
                # and cost of insertion
                ((vhcl_no_insertion, insertion_diff_tour, pos_insertion), insertion_diff_obj) =
                vhcls_pos_insertion[i]

                # Constructing a copied route object
                route_remove_insert_t = route_copy(route_remove_targ)

                # Inserting target t in the considered vehicle's tour
                insert!(route_remove_insert_t.vhcl_tours[vhcl_no_insertion], pos_insertion, t)
                # Updating the dwell time for the target inserted
                route_remove_insert_t.vhcl_dwell_times[vhcl_no_insertion][t] = dwell_time_t
                # Updating the tour cost and objective value of considered vehicle
                new_tour_cost_vhcl = route_remove_insert_t.vhcl_tour_costs[vhcl_no_insertion] + insertion_diff_tour
                new_obj_val_vhcl = route_remove_targ.vhcl_objective_func_costs[vhcl_no_insertion] + insertion_diff_obj
                route_remove_insert_t =
                route_set_vhcl_i_tour_cost(route_remove_insert_t, new_tour_cost_vhcl, new_obj_val_vhcl, vhcl_no_insertion)
                # Checking if a better solution is obtained
                if route_strictly_better(route_remove_insert_t, route)

                    # Running LKH and gradient descent for both vehicles, and returning
                    # the new route object.
                    route = route_copy(route_remove_insert_t)
                    # Updating the dwell times and tours for the two vehicles
                    # TO BE CODED.
                    route=update_dwell_times_tours_costs_obj(route,vhcl_no_insertion,do_exact)
                    route=update_dwell_times_tours_costs_obj(route,vhcl_no_remove,do_exact)
                    return route
                end

            end
        end
    end
    return route

end

function two_pt_move(route,  param_dict) # In this function, two-point swap is performed
    # with "num_vhcls_switch" number of vehicles with least insertion cost using a metric.
    
    num_vhcls_switch=param_dict["num_vhcls_switch"]
    vhcl_removal_choices=param_dict["vhcl_removal_choices"]
    do_exact=false# whether to solve exactly or just use LKH

    if route.instance.dim_depots == 1
        # no vehicles to swap with
        return route
    end
    # Obtaining vehicle according to removal params
    for vhcl_no_remove in choose_vhcls_for_removal(route,vhcl_removal_choices,num_vhcls_switch)
        # Sorting targets in this vehicle in the decreasing difference
        diffs_target_sorted = removal_sorted_vhcl_i(route, vhcl_no_remove)
        
        # print('Vehicle tours are ', route.vhcl_tours)
        # Running through each target in savings_targ and attempting to swap
        # targets
        for ((t, index, t_removal_diff_tour), t_removal_diff_obj) in diffs_target_sorted
            # Obtaining sorted list of vehicles with which a swap will be attempted
            # to be performed. Note that the vehicles have been sorted in the increasing
            # order of insertion cost
            vhcls_pos_insertion = vhcls_sorted_insertion_diff(route, vhcl_no_remove, t)
            # print('Vehicles for insertion for ', t, 'is ', vhcl_nos_insertion, ' costs of insertion are', cost_insertion,\
            #         'and positions for insertion are ', pos_insertion)
        
            # Running through all possible vehicles
            for ((vhcl_no_insertion, t_insertion_diff_tour, t_pos_insertion), t_insertion_diff_obj) in vhcls_pos_insertion
                # Making a copy of the current solution
                route_copy_t_swap = route_copy(route)
                # Removing target from the vehicle with maximum tour cost
                # Checking if the vehicle is covering just one target
                if length(route_copy_t_swap.vhcl_tours[vhcl_no_remove]) == 3
                    route_copy_t_swap.vhcl_tours[vhcl_no_remove] = []
                else
                    deleteat!(route_copy_t_swap.vhcl_tours[vhcl_no_remove],index)
                end
                # removing dwell time
                dwell_time_t=route_copy_t_swap.vhcl_dwell_times[vhcl_no_remove][t]
                route_copy_t_swap.vhcl_dwell_times[vhcl_no_remove][t]=0
                
                # Inserting the considered target in the chosen vehicle's tour.
                # NOTE HERE THAT THE VEHICLE WITH WHICH WE ARE SWAPPING WILL HAVE AT LEAST ONE TARGET
                # BEFOREHAND. HENCE, POS_INSERTION WILL ALWAYS BE NON-ZERO.
                insert!(route_copy_t_swap.vhcl_tours[vhcl_no_insertion],t_pos_insertion,t)
                # updating dwell time as well
                route_copy_t_swap.vhcl_dwell_times[vhcl_no_insertion][t]=dwell_time_t

                # Updating the estimated tour costs of the vehicles
                new_removal_tour_cost_vhcl = route_copy_t_swap.vhcl_tour_costs[vhcl_no_remove] + t_removal_diff_tour
                new_removal_obj_val_vhcl = route_copy_t_swap.vhcl_objective_func_costs[vhcl_no_remove] + t_removal_diff_obj 
                route_copy_t_swap = route_set_vhcl_i_tour_cost(route_copy_t_swap, new_removal_tour_cost_vhcl, new_removal_obj_val_vhcl, vhcl_no_remove)

                new_addition_tour_cost_vhcl=route_copy_t_swap.vhcl_tour_costs[vhcl_no_insertion] + t_insertion_diff_tour
                new_addition_obj_val_vhcl=route_copy_t_swap.vhcl_objective_func_costs[vhcl_no_insertion] + t_insertion_diff_obj 
                route_copy_t_swap = route_set_vhcl_i_tour_cost(route_copy_t_swap,new_addition_tour_cost_vhcl,new_addition_obj_val_vhcl,vhcl_no_insertion)
                
                t_obj_change=t_removal_diff_obj+t_insertion_diff_obj
                # # Performing a 2-opt swap for both the vehicles TODO HERE
                # route_copy_t_swap.two_opt_edges(max_vhcl_no)
                # route_copy_t_swap.two_opt_edges(vhcl_nos_insertion[i])
                # # Obtaining new "saving" obtained for the maximal vehicle, which would
                # # potentially be more than the initial estimated saving after a two-swap
                # saving = route.vhcl_tour_costs[max_vhcl_no] - route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]

                #---------------Rule for going through targets in other vehicle
                # Obtaining sorted list of targets in considered vehicle; sorted based
                # on increasing order of "increase cost" metric for the maximal vehicle.
                # The goal is to swap a target from the maximal vehicle with high savings
                # with a target that has a low increase in cost.
                #println("WARNING IS THIS THE RIGHT SORTING")
                #TODO: UNIMPLEMENTED
                cost_targets_sorted=move_diff_sorted_targets_vhcls_ij(route_copy_t_swap,vhcl_no_insertion,vhcl_no_remove)
                # must return target to swap back, the index, the tour cost of adding, the tour/obj savings from removing
                # dictionary sorted by obj cost of adding

                # print('Sorted targets in other vehicle based on increasing insertion cost is ', increase_cost_t_swap)
                
                # Running through targets in the list
                for ((p, index_p, p_removal_diff_tour,p_removal_diff_obj, p_index_insertion, p_insertion_diff_tour, p_insertion_diff_obj), p_obj_change) in cost_targets_sorted

                    # Checking if increase in cost associated with inserting p in the
                    # maximal vehicle is more than the saving obtained by removing target
                    # t. In this case, the loop is broken.
                    if p_obj_change+t_obj_change<0 || p==t
                        # print('Estimated increase is ', inc, ' and estimated saving is ', saving, '.')
                        break
                    end

                    # Updating the vehicle routes
                    route_copy_t_swap_with_p = route_copy(route_copy_t_swap)

                    # Removing p from vhcl_insertion and inserting it to maximal vehicle.
                    deleteat!(route_copy_t_swap_with_p.vhcl_tours[vhcl_no_insertion],index_p)
                    dwell_time_p=route_copy_t_swap_with_p.vhcl_dwell_times[vhcl_no_insertion][p]
                    route_copy_t_swap_with_p.vhcl_dwell_times[vhcl_no_insertion][p]=0.

                    # Checking if the maximal vehicle has an empty tour or not
                    if length(route_copy_t_swap_with_p.vhcl_tours[vhcl_no_remove]) < 3
                        depot=collect(keys(route_copy_t_swap_with_p.instance.depots))[vhcl_no_remove]
                        route_copy_t_swap_with_p.vhcl_tours[vhcl_no_remove] = [depot, p, depot]
                    else
                        insert!(route_copy_t_swap_with_p.vhcl_tours[vhcl_no_remove],p_index_insertion,p)
                    end
                    route_copy_t_swap_with_p.vhcl_dwell_times[vhcl_no_remove][p]=dwell_time_p

                    # Updating the tour costs for the two vehicles
                    new_new_removal_tour_cost_vhcl=route_copy_t_swap_with_p.vhcl_tour_costs[vhcl_no_insertion]+p_removal_diff_tour
                    new_new_removal_obj_val_vhcl=route_copy_t_swap_with_p.vhcl_objective_func_costs[vhcl_no_insertion]+p_removal_diff_obj
                    route_set_vhcl_i_tour_cost(route_copy_t_swap_with_p,new_new_removal_tour_cost_vhcl,new_new_removal_obj_val_vhcl,vhcl_no_insertion)

                    
                    new_new_insertion_tour_cost_vhcl=route_copy_t_swap_with_p.vhcl_tour_costs[vhcl_no_remove]+p_insertion_diff_tour
                    new_new_insertion_obj_val_vhcl=route_copy_t_swap_with_p.vhcl_objective_func_costs[vhcl_no_remove]+p_insertion_diff_obj
                    route_set_vhcl_i_tour_cost(route_copy_t_swap_with_p,new_new_insertion_tour_cost_vhcl,new_new_insertion_obj_val_vhcl,vhcl_no_remove)


                    # Performing a two-opt edge swap for the two vehicles
                    #route_copy_t_swap_with_p.two_opt_edges(max_vhcl_no)
                    #route_copy_t_swap_with_p.two_opt_edges(vhcl_nos_insertion[i])

                    # Checking if the obtained solution is better than the current solution
                    if route_strictly_better(route_copy_t_swap_with_p,route)
                        
                        
                        #println(p_obj_change,t_obj_change)
                        #println("SWAPPING: ",t," AND ",p," BETWEEN ",vhcl_no_remove," AND ",vhcl_no_insertion)
                        # The tours of the maximal vehicle and the other vehicle are
                        # improved by using LKH
                        # print('Better solution obtained.')
                        route = route_copy(route_copy_t_swap_with_p)
                        
                        route=update_dwell_times_tours_costs_obj(route,vhcl_no_insertion,do_exact)
                        route=update_dwell_times_tours_costs_obj(route,vhcl_no_remove,do_exact)
                        return route 
                    end
                end
            end
        end
    end
    return route
end

function perturbation(route, counter, local_search_moves, local_search_params,
    shaking_phase, angle_perturb_depots = [], avg_distance = [],debug=false)
    # In this function, the perturbation stage similar to the MD algorithm is implemented.
    if debug
        println("Perturbation number is ", counter)
    end

    # Checking if counter is 0
    if counter == 0

        # Generating a random angle for perturbation for each depot
        angle_perturb_depots = [rand()*2*pi for i in 1:route.instance.dim_depots]

        # Obtaining the average distance of each depot from the targets it is connected
        # to in the current solution.
        # Note that for each vehicle's route, the first and last nodes visited
        # are the depot
        avg_distance = []
        for i in 1:route.instance.dim_depots

            # Getting the sum cost to travel between the depot and the
            # two targets that are incident on it.
            if isempty(route.vhcl_tours[i]) # Checking if tour is empty
                sum_cost_i = 0
            else
                # Obtaining the depot index of the considered vehicle
                dep_ind = route.instance.dim_targets + i
                # Obtaining the first target and last target visited by the vehicle
                t_first = route.vhcl_tours[i][2]; t_last = route.vhcl_tours[i][end - 1]
                sum_cost_i = route.instance.cost_traversal[t_first][dep_ind]
                 + route.instance.cost_traversal[t_last][dep_ind]
            end
            # Updating the average distance
            append!(avg_distance, sum_cost_i/2)

        end

    end

    # Checking if the angle of perturbation of depot or average distance array is
    # empty, which would occur if such arguments were not passed as input to the function
    if isempty(avg_distance) || isempty(angle_perturb_depots)
        error("Average distance or angle of perturbation is not passed.")
    end

    # Obtaining the perturbed depot locations for passed counter value
    depots_perturbed = OrderedCollections.OrderedDict()
    for i in 1:route.instance.dim_depots

        # Obtaining the location of the old depot, and the node number corresponding
        # to the depot, for vehicle number i
        depot_node_no = route.instance.dim_targets + i
        coords_old_depot = route.instance.depots[depot_node_no]

        # Obtaining the angle of perturbation for the considered depot. If counter = 0,
        # then the generated angle of perturbation for considered depot would be used.
        # For every subsequent counter value, the angle of perturbation would be 144 deg in
        # addition to previous angle considered.
        angle_perturb = mod((angle_perturb_depots[i] + counter*4*pi/5), 2*pi)

        # Calculating the coordinates of the new depot
        depots_perturbed[depot_node_no] = (coords_old_depot[1] + avg_distance[i]*cos(angle_perturb),
         coords_old_depot[2] + avg_distance[i]*sin(angle_perturb))

    end

    # Ordering the perturbed depots
    depots_perturbed = sort(depots_perturbed)

    # Making a new data object
    data_perturbed_depots = deepcopy(route.instance.data)
    # Making a new instance with perturbed depot locations
    instance_perturbed_depots = Instance(data_perturbed_depots, route.instance.alpha,
     route.instance.tau)
    
    # Making a new route object with replaced instance, but same target allocations
    # and dwell times
    route_perturbed_depots = Route(instance_perturbed_depots)

    # Running through each vehicle
    for i in 1:route.instance.dim_depots

        # Obtaining the list of targets visited by the ith vehicle
        targ_covered = route.vhcl_tours[i][2:end - 1]
        # Running the single vehicle function to obtain the optimal TSP tour
        # and dwell times.
        vhcl_tour, dwell_times, tour_cost, targ_covered_arr =
         single_vehicle_TSP_dwell_times(instance_perturbed_depots, targ_covered, i, false)
        
        # Setting the dwell time for the considered vehicle
        for j in eachindex(targ_covered_arr)
            route_perturbed_depots.vhcl_dwell_times[i][targ_covered_arr[j]] = dwell_times[j]
        end

        # Setting the vehicle tour and tour cost
        # Obtaining the objective of the considered vehicle
        obj_vhcl_i =
         obtain_vhcl_objective_value(tour_cost, dwell_times,
          route_perturbed_depots.instance.alpha, route_perturbed_depots.instance.tau)
        route_perturbed_depots =
         route_set_vhcl_i_tour_tour_cost(route_perturbed_depots, vhcl_tour, tour_cost, obj_vhcl_i, i)

    end

    # Performing a local search on the obtained solution in the perturbed graph
    j, j_max = 1, length(local_search_moves)+1
    while j < j_max
        x__ = local_search_moves[j](route_perturbed_depots, local_search_params[j])
        # Checking if the current solution is better than previous incumbent solution
        if route_strictly_better(x__, route_perturbed_depots)
            route_perturbed_depots = x__
            j = 1
        else # Moving to the next neighborhood
            j += 1
        end
    end

    # Using the obtained target allocation to each vehicle from the perturbed solution
    # in the original graph
    route_new_soln_orig_graph = route_copy(route)
    # Obtaining tours for each vehicle using the target allocation from the
    # perturbed solution
    for i in 1:route.instance.dim_depots

        # Obtaining the targets covered by the ith vehicle
        targ_covered = route_perturbed_depots.vhcl_tours[i][2:end - 1]
        # Running the single vehicle TSP and dwell time function
        vhcl_tour, dwell_times, tour_cost, targ_covered_arr =
         single_vehicle_TSP_dwell_times(route.instance, targ_covered, i, false)
        # Setting the dwell time of the vehicle
        for j in eachindex(targ_covered_arr)
            route_new_soln_orig_graph.vhcl_dwell_times[i][targ_covered_arr[j]] = dwell_times[j]
        end

        # Setting the vehicle tour and tour cost
        # Obtaining the objective of the considered vehicle
        obj_vhcl_i =
         obtain_vhcl_objective_value(tour_cost, dwell_times,
         route_new_soln_orig_graph.instance.alpha, route_new_soln_orig_graph.instance.tau)
        route_new_soln_orig_graph =
         route_set_vhcl_i_tour_tour_cost(route_new_soln_orig_graph, vhcl_tour, tour_cost, obj_vhcl_i, i)

    end

    return route_new_soln_orig_graph, angle_perturb_depots, avg_distance

end