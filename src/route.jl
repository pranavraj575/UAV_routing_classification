using Classes
include("Instance.jl")
include("highs_single_vehicle_TSP_dwelltime_functions.jl")

# In this function, a route object is constructed corresponding to the considered
# graph

@class mutable Route begin
    instance::AbstractInstance
    vhcl_tours::Dict
    vhcl_tour_costs::Dict
    vhcl_objective_func_costs::Dict
    vhcl_dwell_times::Dict
    soln_cost::Float64

    function Route(instance::AbstractInstance)
        
        # Initializing with zero cost for each vehicle, zero dwell times, and
        # no tours for any vehicle
        vhcl_tours = Dict(); vhcl_dwell_times = Dict(); vhcl_tour_costs = Dict();
        vhcl_objective_func_costs = Dict();

        for i = 1: instance.data.dim_depots

            vhcl_tours[i] = []
            vhcl_dwell_times[i] = Dict()
            vhcl_tour_costs[i] = 0
            vhcl_objective_func_costs[i] = 0
            # Running through each target
            for t in keys(instance.targets)
                vhcl_dwell_times[i][t] = 0
            end

        end

        return new(instance, vhcl_tours, vhcl_tour_costs, vhcl_objective_func_costs,
         vhcl_dwell_times, 0)

    end

end

function reset_dwell_times_vhcl(route,vhcl_no)
    for t in keys(route.instance.targets)
        route.vhcl_dwell_times[vhcl_no][t] = 0.
    end
end

function route_copy(route_to_be_copied)    
    # Making a copy of the route object

    # Creating a new route object
    new_route_object = Route(route_to_be_copied.instance)
    # Updating the elements of the route object
    new_route_object.vhcl_tours = deepcopy(route_to_be_copied.vhcl_tours)
    new_route_object.vhcl_dwell_times = deepcopy(route_to_be_copied.vhcl_dwell_times)
    new_route_object.soln_cost = deepcopy(route_to_be_copied.soln_cost)
    new_route_object.vhcl_tour_costs = deepcopy(route_to_be_copied.vhcl_tour_costs)
    new_route_object.vhcl_objective_func_costs = deepcopy(route_to_be_copied.vhcl_objective_func_costs)

    return new_route_object

end

function sorted_vhcls(route,sorting_fun)
    # In this function, sort vehicle numbers
    # NOTE: Vehicles that have zero targets are not considered
    dic=Dict()
    for i in 1:route.instance.dim_depots
        if length(route.vhcl_tours[i])>2
            dic[i]=sorting_fun(route,i)
        end
    end
    return collect(keys(sort(dic; byvalue=true)))
end

function sorted_vhcls_most_targets(route)
    # In this function, sort vehicle numbers by number of targets
    # NOTE: Vehicles that have zero targets are not considered
    function sorting_fun(rte,vhcl_no)
        num_targets=length(rte.vhcl_tours[vhcl_no])-2
        return -num_targets # since highest first
    end
    return sorted_vhcls(route,sorting_fun)
end

function sorted_vhcls_longest_tour(route)
    # In this function, sort vehicle numbers by tour cost
    # NOTE: Vehicles that have zero targets are not considered
    function sorting_fun(rte,vhcl_no)
        tour_cost=rte.vhcl_tour_costs[vhcl_no]
        return -tour_cost # since highest first
    end
    return sorted_vhcls(route,sorting_fun)
end

function sorted_vhcls_largest_obj_val(route)
    # In this function, the sort vhcl numbers by largest obj value
    # NOTE: Vehicles that have zero targets are not considered
    function sorting_fun(rte,vhcl_no)
        objective=rte.vhcl_objective_func_costs[vhcl_no]
        return -objective # since highest first
    end
    return sorted_vhcls(route,sorting_fun)
end

function sorted_vhcls_smallest_obj_val(route)
    # In this function, the sort vhcl numbers by smallest obj value
    # NOTE: Vehicles that have zero targets are not considered
    function sorting_fun(rte,vhcl_no)
        objective=rte.vhcl_objective_func_costs[vhcl_no]
        return objective # since highest first
    end
    return sorted_vhcls(route,sorting_fun)
end


function obtain_vhcl_objective_value(tour_cost, dwell_times, alpha, tau)
    # In this function, the objective value of a particular value is obtained
    # given dwell time and tour cost. TO BE CODED.
    if length(dwell_times)==0
        return 0
    end
    G=undiscounted_info_gain_func([dwell_times[key] for key in keys(dwell_times)],
                                    [tau for _ in keys(dwell_times)])
    obj_val=exp(-alpha*tour_cost)*exp(-alpha*sum(dwell_times))*G
    return obj_val

end

function route_set_vhcl_i_tour_tour_cost(route, tour, tour_cost, vhcl_objective_value, vhcl_no)
    # In this function, the tour and tour cost is set for a particular vehicle.
    # Further, the objective value for the considered vehicle is also set for the same 
    # vehicle.
    # Here, vehicle tour is an array.

    route.vhcl_tours[vhcl_no] = tour
    route.vhcl_tour_costs[vhcl_no] = tour_cost
    
    # Obtaining the objective function value for the vehicle, and updating the
    # objective function cost of the route
    # Obtaining the previous objective cost for the considered vehicle
    vhcl_no_prev_cost = route.vhcl_objective_func_costs[vhcl_no]
    # Updating the objective function value for the considered vehicle
    route.vhcl_objective_func_costs[vhcl_no] = vhcl_objective_value
    # Updating the route objective value
    route.soln_cost += vhcl_objective_value - vhcl_no_prev_cost
    return route
end

function route_set_vhcl_i_tour_cost(route, tour_cost, vhcl_objective_value, vhcl_no)
    # This function is similar to the previous function, except that the tour
    # for the considered vehicle is not set. It is assumed that the route for the
    # considered vehicle has already been previously updated.

    route.vhcl_tour_costs[vhcl_no] = tour_cost
    
    # Obtaining the objective function value for the vehicle, and updating the
    # objective function cost of the route
    # Obtaining the previous objective cost for the considered vehicle
    vhcl_no_prev_cost = route.vhcl_objective_func_costs[vhcl_no]
    # Obtaining the objective function value
    route.vhcl_objective_func_costs[vhcl_no] = vhcl_objective_value
    # Updating the route objective value
    route.soln_cost += vhcl_objective_value - vhcl_no_prev_cost

    return route
end

function route_set_vhcl_i_dwell_times(route, dwell_times, vhcl_objective_value, vhcl_no)
    # This function is similar to the previous function, it sets dwell times
    # dwell times should be a dictionary of keys to dwell times

    for key in keys(dwell_times)
        route.vhcl_dwell_times[vhcl_no][key] = dwell_times[key]
    end
    
    # Obtaining the objective function value for the vehicle, and updating the
    # objective function cost of the route
    # Obtaining the previous objective cost for the considered vehicle
    vhcl_no_prev_cost = route.vhcl_objective_func_costs[vhcl_no]
    # Obtaining the objective function value
    route.vhcl_objective_func_costs[vhcl_no] = vhcl_objective_value
    # Updating the route objective value
    route.soln_cost += vhcl_objective_value - vhcl_no_prev_cost

    return route
end

function removal_difference_estimate_vhcl_i(route, vhcl_no, targ_remove)
    # In this function, the change associated with removing a particular target
    # from the vehicle route is obtained. Further, the diff associated with the
    # tour cost is also returned. CODE TO BE UPDATED.

    # NOTE: this will return new - old for both the tour difference and objective difference

    # tour that the vehicle has
    tour=route.vhcl_tours[vhcl_no]

    # index of target on tour
    targ_index=findall(x->x==targ_remove, tour)[1]

    # alpha
    alpha=route.instance.alpha

    # original objective cost of this vehicle
    J=route.vhcl_objective_func_costs[vhcl_no]

    # original info gain G of this vehicle
    G=get_vhcl_info_gain(route,vhcl_no)

    # dwell time of targ_remove (assuming these do not change)
    d=route.vhcl_dwell_times[vhcl_no][targ_remove]

    # tour going from a => targ_remove => b
    # to a => b
    a=tour[targ_index-1]
    b=tour[targ_index+1]

    # adding a => b, removing a => targ_remove and targ_remove => b
    length_change = (route.instance.cost_traversal[a][b]-
                        route.instance.cost_traversal[a][targ_remove]-
                        route.instance.cost_traversal[targ_remove][b]
                    )
    # multiplying J by this corrects the TSP path cost solution
    factor_1=exp(-alpha*length_change)
    # multiplying J by this corrects the discount term
    factor_2=exp(alpha*d)
    # multiplying by this corrects the info gain term
    factor_3=(G-Ii(d,route.instance.tau))/G

    # NEW - OLD
    objective_change=J*factor_1*factor_2*factor_3 - J

    return objective_change, length_change

end

function removal_sorted_vhcl_i(route, vhcl_no)
    # In this function, the sorted list of targets in the order of decreasing difference
    # for the considered vehicle is obtained

    # Running through all targets in the considered vehicle's tour. NOTE THAT
    # THE VEHICLE'S TOUR HAS THE FIRST AND LAST INDEX TO BE THE DEPOT.
    obj_change_dict = Dict()
    for i in 2:length(route.vhcl_tours[vhcl_no]) - 1
        # Obtaining changes for the considered targets
        targ=route.vhcl_tours[vhcl_no][i]
        obj_change, tour_change = removal_difference_estimate_vhcl_i(route, vhcl_no, targ)
        # Adding to the dictionary the target removed and the index of removal
        obj_change_dict[(targ, i, tour_change)] = obj_change
    end
    # Sorting the dictionary in the DECREASING order of change, and returning the
    # same
    return sort(obj_change_dict, rev = true; byvalue=true)

end

function get_vhcl_info_gain(route,vhcl_no)
    vertices=route.vhcl_tours[vhcl_no]
    vertices=vertices[2:length(vertices)-1]
    return undiscounted_info_gain_func([route.vhcl_dwell_times[vhcl_no][k] for k in vertices],
                                        [route.instance.tau for _ in vertices])
end

function insertion_difference_estimate_targ(route, vhcl_no_insertion, targ, dwell_time)
    # In this function, the difference of insertion of a target in a considered vehicle
    # is obtained. Further, the position of best insertion is also obtained.
    # TO BE CODED.
    # NOTE: dwell time of the target to be inserted is probably needed

    # NOTE: this will return new - old for both tour difference and objective difference 

    best_insertion = 1
    best_insertion_diff_tour = 0
    best_insertion_diff_obj = -Inf

    tour=route.vhcl_tours[vhcl_no_insertion]

    # original objective of this vehicle
    J=route.vhcl_objective_func_costs[vhcl_no_insertion]

    # original info gain G of this vehicle
    G=get_vhcl_info_gain(route,vhcl_no_insertion)

    # dwell time of targ (assuming these do not change)
    d=dwell_time

    alpha=route.instance.alpha
    for pos_insertion in 2:length(tour)
        # inserting node targ into the segment of tour a => b
        # ending with a => targ => b
        a=tour[pos_insertion-1]
        b=tour[pos_insertion]
        
        # removing a => b, adding a => targ and b => targ
        length_change = (route.instance.cost_traversal[a][targ]+
                            route.instance.cost_traversal[targ][b]-
                            route.instance.cost_traversal[a][b]
                        )

        # multiplying J by this corrects the TSP path cost solution
        factor_1=exp(-alpha*length_change)
        # multiplying J by this corrects the discount term
        factor_2=exp(-alpha*d)
        # multiplying by this corrects the info gain term
        factor_3=(G+Ii(d,route.instance.tau))/G


        # NEW MINUS OLD
        objective_change=J*factor_1*factor_2*factor_3 - J

        # new J minus original J
        insertion_diff_obj = objective_change

        # if this difference is better than the best we've found, update
        if insertion_diff_obj>best_insertion_diff_obj
            best_insertion_diff_obj=insertion_diff_obj
            # length change is the tour insertion diff
            best_insertion_diff_tour=length_change
            best_insertion=pos_insertion
        end
    end

    return best_insertion_diff_obj, best_insertion_diff_tour, best_insertion

end

function vhcls_sorted_insertion_diff(route, vhcl_no_least_obj_val, targ)
    # In this function, the vehicles are sorted from maximum to minimum insertion
    # diffs for a given target

    # Running through all vehicles, and checking that the vehicle from which
    # the target is removed is not the vehicle that is considered
    vhcls_insertion_diff = Dict()
    targ_dwell_time=route.vhcl_dwell_times[vhcl_no_least_obj_val][targ]

    for i = 1: route.instance.dim_depots
        if i != vhcl_no_least_obj_val
            insertion_diff_obj, insertion_diff_tour, pos_insertion = insertion_difference_estimate_targ(route, i, targ,targ_dwell_time)
            vhcls_insertion_diff[(i, insertion_diff_tour, pos_insertion)] = insertion_diff_obj
        end
    end
    # Sorting the vehicles in the DECREASING order of insertion diff. NOTE: 
    # THE RETURNED VARIABLE IS AN ARRAY
    return collect(sort(vhcls_insertion_diff, rev = true; byvalue=true))

end

function move_diff_estimate_vhcls_ij(route, vhcl_removal,vhcl_insertion, targ_remove)
    dwell_time=route.vhcl_dwell_times[vhcl_removal][targ_remove]
    removal_diff_obj,removal_diff_tour=removal_difference_estimate_vhcl_i(route, vhcl_removal, targ_remove)
    insertion_diff_obj, insertion_diff_tour, index_insertion=insertion_difference_estimate_targ(route, vhcl_insertion, targ_remove, dwell_time)
    return removal_diff_tour,removal_diff_obj,insertion_diff_tour,insertion_diff_obj,index_insertion
end

function move_diff_sorted_targets_vhcls_ij(route,vhcl_removal,vhcl_insertion)
    # return sorted dictionary targets in vhcl_removal by cost of moving to vhcl_insertion
    # for target p, this dictionary entry is 
    # (p, index_p, p_removal_diff_tour,p_removal_diff_obj, p_index_insertion, p_insertion_diff_tour, p_insertion_diff_obj)=> p_obj_change
    # NOTE: this will return new - old for both tour difference and objective difference 

    obj_change_dict = Dict()
    for i in 2:length(route.vhcl_tours[vhcl_removal]) - 1
        # Obtaining changes for the considered targets
        targ=route.vhcl_tours[vhcl_removal][i]
        removal_diff_tour,removal_diff_obj,insertion_diff_tour,insertion_diff_obj,index_insertion = move_diff_estimate_vhcls_ij(route, vhcl_removal,vhcl_insertion, targ)
        obj_change_dict[(targ, 
                                i, removal_diff_tour,removal_diff_obj,
                                index_insertion, insertion_diff_tour, insertion_diff_obj,
                            )] = insertion_diff_obj+removal_diff_obj #overall objective change
    end
    # Sorting the dictionary in the DECREASING order of change, and returning the
    # same
    return sort(obj_change_dict, rev = true; byvalue=true)
end

function route_strictly_better(route_new, route_old)
    # In this function, a new route object is compared with an older route object.
    # If the new solution is better than the older solution, true is returned, else false.
    if route_new.soln_cost > route_old.soln_cost
        return true
    else
        return false
    end
end