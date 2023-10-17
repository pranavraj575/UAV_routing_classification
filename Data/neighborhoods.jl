using OrderedCollections
include("route.jl")
include("instance.jl")
include("single_vehicle_TSP_dwelltime_functions.jl")

function one_pt_move(route, num_vhcls_switch = 1)
    # In this function, a target switch from the vehicle with the least objective
    # value to a vehicles sorted in increasing order of insertion cost is performed.

    # Obtaining vehicle with the least objective value
    vhcl_no_least_val = identifying_vhcl_least_obj_val(route)
    # Sorting targets in this vehicle in the increasing order of savings
    savings_target_sorted = savings_sorted_vhcl_i(route, vhcl_no_least_val)

    # Running through each target in savings_target_sorted and attempting to
    # remove targets with maximal potential savings from the tour
    for ((t, index, saving_tour), saving_obj) in savings_target_sorted

        # Making a copy of the object
        route_remove_targ = route_copy(route)
        # Removing target t from the vehicle with the least objective value, which
        # is located at "index" in the vehicle's tour
        deleteat!(route_remove_targ.vhcl_tours[vhcl_no_least_val], index)
        # Checking if the vehicle with least objective value is not covering any
        # more targets. This occurs when the vehicle initially covered just one target.
        # Now, checking if the considered vehicle has a tour of length 2, which 
        # essentially means that the tour of the vehicle is from its depot back to the depot.
        if length(route_remove_targ.vhcl_tours[vhcl_no_least_val]) == 2
            route_remove_targ.vhcl_tours[vhcl_no_least_val] = []
        end

        # Updating the dwell time dictionary for the target removed from the considered
        # vehicle
        dwell_time_t = route_remove_targ.vhcl_dwell_times[vhcl_no_least_val][t]
        route_remove_targ.vhcl_dwell_times[vhcl_no_least_val][t] = 0
        # Updating the tour cost and objective value of considered vehicle
        new_tour_cost_vhcl = route_remove_targ.vhcl_tour_costs[vhcl_no_least_val] - saving_tour
        new_obj_val_vhcl = route_remove_targ.vhcl_objective_func_costs[vhcl_no_least_val] - saving_obj
        route_remove_targ =
         route_set_vhcl_i_tour_cost(route_remove_targ, new_tour_cost_vhcl, new_obj_val_vhcl, vhcl_no_least_val)

        # Identifying the sorted list of vehicles in increasing order of insertion
        # cost metric
        vhcls_pos_insertion = vhcls_sorted_insertion_cost(route, vhcl_no_least_val, t)

        # Running through the top num_vhcls_switch number of vehicles
        for i in 1:num_vhcls_switch
            # Obtaining the chosen vehicle index for insertion, position of insertion,
            # and cost of insertion
            ((vhcl_no_insertion, insertion_cost_tour, pos_insertion), insertion_cost_obj) =
             vhcls_pos_insertion[i]

            # Constructing a copied route object
            route_remove_insert_t = route_copy(route_remove_targ)

            # Inserting target t in the considered vehicle's tour
            insert!(route_remove_insert_t.vhcl_tours[vhcl_no_insertion], pos_insertion, t)
            # Updating the dwell time for the target inserted
            route_remove_insert_t.vhcl_dwell_times[vhcl_no_insertion][t] = dwell_time_t
            # Updating the tour cost and objective value of considered vehicle
            new_tour_cost_vhcl = route_remove_insert_t.vhcl_tour_costs[vhcl_no_insertion] + insertion_cost_tour
            new_obj_val_vhcl = route_remove_targ.vhcl_objective_func_costs[vhcl_no_insertion] + insertion_cost_obj
            route_remove_insert_t =
             route_set_vhcl_i_tour_cost(route_remove_insert_t, new_tour_cost_vhcl, new_obj_val_vhcl, vhcl_no_insertion)

            # Checking if a better solution is obtained
            if route_strictly_better(route_remove_insert_t, route)

                # Running LKH and gradient descent for both vehicles, and returning
                # the new route object.
                route = route_copy(route_remove_insert_t)
                # Updating the dwell times and tours for the two vehicles
                # TO BE CODED.

                return route

            end

        end

    end

    return route

end

function two_pt_move(route, parameter)

end

function perturbation(route, counter, local_search_moves, local_search_params,
    shaking_phase, angle_perturb_depots = [], avg_distance = [])
    # In this function, the perturbation stage similar to the MD algorithm is implemented.

    println("Perturbation number is ", counter)

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
        vhcl_tour, dwell_time, tour_cost, targ_covered_arr =
         single_vehicle_TSP_dwell_times(instance_perturbed_depots, targ_covered, i, false)
        
        # Setting the dwell time for the considered vehicle
        for j in eachindex(targ_covered_arr)
            route_perturbed_depots.vhcl_dwell_times[i][targ_covered_arr[j]] = dwell_time[j]
        end

        # Setting the vehicle tour and tour cost
        # Obtaining the objective of the considered vehicle
        obj_vhcl_i =
         obtain_vhcl_objective_value(route_perturbed_depots, route_perturbed_depots.vhcl_dwell_times[i],
          route_perturbed_depots.instance.alpha, route_perturbed_depots.instance.tau)
        route_perturbed_depots =
         route_set_vhcl_i_tour_tour_cost(route_perturbed_depots, vhcl_tour, tour_cost, obj_vhcl_i, i)

    end

    # Performing a local search on the obtained solution in the perturbed graph
    j, j_max = 0, length(local_search_moves)
    while j < j_max
        x__ = local_search_moves[j](route_perturbed_depots, local_search_params[j])
        # Checking if the current solution is better than previous incumbent solution
        if route_strictly_better(x__, route_perturbed_depots)
            route_perturbed_depots = x__
            j = 0
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
        vhcl_tour, dwell_time, tour_cost, targ_covered_arr =
         single_vehicle_TSP_dwell_times(route.instance, targ_covered, i, false)
        # Setting the dwell time of the vehicle
        for j in eachindex(targ_covered_arr)
            route_new_soln_orig_graph.vhcl_dwell_times[i][targ_covered_arr[j]] = dwell_time[j]
        end

        # Setting the vehicle tour and tour cost
        # Obtaining the objective of the considered vehicle
        obj_vhcl_i =
         obtain_vhcl_objective_value(route_new_soln_orig_graph, route_new_soln_orig_graph.vhcl_dwell_times[i],
         route_new_soln_orig_graph.instance.alpha, route_new_soln_orig_graph.instance.tau)
        route_new_soln_orig_graph =
         route_set_vhcl_i_tour_tour_cost(route_new_soln_orig_graph, vhcl_tour, tour_cost, obj_vhcl_i, i)

    end

    return route_new_soln_orig_graph, angle_perturb_depots, avg_distance

end