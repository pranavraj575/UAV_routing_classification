using Classes
include("Instance.jl")

# In this function, a route object is constructed corresponding to the considered
# graph

@class Route begin
    instance::AbstractInstance
    vhcl_tours::Dict
    vhcl_tour_costs::Dict
    vhcl_objective_func_costs::Dict
    vhcl_dwell_times::Dict
    soln_cost::Float16

    function Route(instance::AbstractInstance)
        
        # Initializing with zero cost for each vehicle, zero dwell times, and
        # no tours for any vehicle
        vhcl_tours = Dict(); vhcl_dwell_times = Dict(); vhcl_tour_costs = Dict();
        vhcl_objective_func_costs = Dict();

        for i = 1: instance.data.dim_depots

            vhcl_tours[i] = Dict()
            vhcl_dwell_times[i] = Dict()
            vhcl_tour_costs[i] = 0
            vhcl_objective_func_costs[i] = 0
            # Running through each target
            for t in keys(instance.targets)
                vhcl_dwell_times[i][t] = 0
            end

        end

        return new(instance, vhcl_tours, vhcl_dwell_times, 0)

    end

end

function route_copy(route_to_be_copied)    
    # Making a copy of the route object

    # Creating a new route object
    new_route_object = Route(route_to_be_copied.instance)
    # Updating the elements of the route object
    new_route_object.vhcl_tours = deepcopy(route_to_be_copied.vhcl_tours)
    new_route_object.dwell_times = deepcopy(route_to_be_copied.dwell_times)
    new_route_object.soln_cost = deepcopy(route_to_be_copied.soln_cost)
    new_route_object.vhcl_tour_costs = deepcopy(route_to_be_copied.vhcl_tour_costs)

    return new_route_object

end

function identifying_vhcl_least_obj_val(route)
    # In this function, the vehicle with the least objective value is returned
    return collect(keys(route.vhcl_objective_func_costs))[indmin(collect(values(route.vhcl_objective_func_costs)))]

end

function obtain_vhcl_objective_value(tour_cost, dwell_time, alpha, tau)
    # In this function, the objective value of a particular value is obtained
    # given dwell time and tour cost. TO BE CODED.

    obj_val = 0
    return obj_val

end

function route_set_vhcl_i_tour_dwell_time(route, tour, tour_cost, vhcl_objective_value,
     dwell_time_dict, vhcl_no)
    # In this function, the tour, tour cost, dwell times is set for a particular vehicle.
    # Further, the objective value for the considered vehicle is also set for the same 
    # vehicle.
    # Here, dwell time is passed as a dictionary, whereas vehicle tour is an array.

    route.vhcl_tours[vhcl_no] = tour
    route.vhcl_tour_costs[vhcl_no] = tour_cost
    # Running through the dwell times
    for (i, val) in dwell_time_dict
        route.vhcl_dwell_times[vhcl_no][i] = val
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

function route_set_vhcl_i_tour_cost_dwell_time(route, tour_cost, vhcl_objective_value,
    dwell_time_dict, vhcl_no)
    # This function is similar to the previous function, except that the tour
    # for the considered vehicle is not set. It is assumed that the route for the
    # considered vehicle has already been previously updated.

    route.vhcl_tour_costs[vhcl_no] = tour_cost
    # Running through the dwell times
    for (i, val) in dwell_time_dict
        route.vhcl_dwell_times[vhcl_no][i] = val
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

function savings_target_vhcl_i(route, vhcl_no, targ_remove)
    # In this function, the savings associated with removing a particular target
    # from the vehicle route is obtained. CODE TO BE UPDATED.

    savings = 0

end

function savings_sorted_vhcl_i(route, vhcl_no)
    # In this function, the sorted list of targets in the order of decreasing saving
    # for the considered vehicle is obtained

    # Running through all targets in the considered vehicle's tour. NOTE THAT
    # THE VEHICLE'S TOUR HAS THE FIRST AND LAST INDEX TO BE THE DEPOT.
    savings_dict = Dict()
    for i in 2:length(route.vhcl_tours[vhcl_no]) - 1
        # Obtaining savings for the considered targets
        savings = savings_target_vhcl_i(route, vhcl_no, route.vhcl_tours[vhcl_no][i])
        # Adding to the dictionary
        savings_dict[route.vhcl_tours[vhcl_no][i]] = savings
    end
    # Sorting the dictionary in the decreasing order of savings, and returning the
    # same
    return sort(savings_dict, rev = true; byvalue=true)

end

function insertion_cost_estimate_targ(route, vhcl_no_insertion, targ)
    # In this function, the cost of insertion of a target in a considered vehicle
    # is obtained. Further, the position of cheapest insertion is also obtained.
    # TO BE CODED.

    pos_insertion = 1
    insertion_cost = 0

    return insertion_cost, pos_insertion

end

function vhcls_sorted_insertion_cost(route, vhcl_no_least_obj_val, targ)
    # In this function, the vehicles are sorted from minimum to maximum insertion
    # costs for a given target

    # Running through all vehicles, and checking that the vehicle from which
    # the target is removed is not the vehicle that is considered
    vhcls_insertion_cost = Dict()

    for i = 1: length(route.vhcl_tours)
        if i != vhcl_no_least_obj_val
            insertion_cost, pos_insertion = insertion_cost_estimate_targ(route, i, targ)
            vhcls_insertion_cost[i] = (insertion_cost, pos_insertion)
        end
    end
    # Sorting the vehicles in the increasing order of insertion cost
    return sort(vhcls_insertion_cost; byvalue=true)

end