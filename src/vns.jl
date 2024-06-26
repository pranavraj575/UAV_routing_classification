# variable neighborhood search
# creates a solution by local search, through repeated 'shaking' a solution then performing local search

using Classes
include("route.jl")
include("instance.jl")
include("construct.jl")
include("neighborhoods.jl")

function construct_solution(instance, mode = "min_sum")
    # Construction heuristic for initial partition to be used here
    return partition_tours_star_heuristic(instance)
end

function shaking(route, counter, local_search_moves, local_search_parameters,
     shaking_phase, angle_perturb_depots = [], avg_distance = [])
    return perturbation(route,counter,local_search_moves,local_search_parameters,
    shaking_phase,angle_perturb_depots,avg_distance)
    # In this function, a shaking step is described.
end

function check_if_valid(route)
    for target in collect(keys(route.instance.targets))
        found=false
        for vhcl_no in 1:route.instance.dim_depots
            if target in route.vhcl_tours[vhcl_no]
                if found # no repeats
                    return false
                end
                found=true
            end
        end
        if !found # target not in any routes
            return false
        end
    end
    return true
end

function solver_to_dict(solver)
    init_sol=solver.initial_solution
    local_sol=solver.local_search_solution
    final_sol=solver.final_solution
    
    data=solver.instance.data

    return Dict(

        "initial_solution"=>[init_sol.vhcl_tours[i] for i in 1: data.dim_depots],
        "initial_tour_costs"=>[init_sol.vhcl_tour_costs[i] for i in 1: data.dim_depots],
        "initial_dwell_times"=>[[init_sol.vhcl_dwell_times[i][t] 
                                    for t in init_sol.vhcl_tours[i][2:length(init_sol.vhcl_tours[i])-1]] 
                                for i in 1: data.dim_depots],
        "initial_obj"=>init_sol.soln_cost,
        "initial_time"=>solver.initial_time,

        "local_search_solution"=>[local_sol.vhcl_tours[i] for i in 1: data.dim_depots],
        "local_search_tour_costs"=>[local_sol.vhcl_tour_costs[i] for i in 1: data.dim_depots],
        "local_search_dwell_times"=>[[local_sol.vhcl_dwell_times[i][t] 
                                    for t in local_sol.vhcl_tours[i][2:length(local_sol.vhcl_tours[i])-1]] 
                                for i in 1: data.dim_depots],
        "local_search_obj"=>local_sol.soln_cost,
        "local_search_time"=>solver.local_search_time,

        "final_solution"=>[final_sol.vhcl_tours[i] for i in 1: data.dim_depots],
        "final_tour_costs"=>[final_sol.vhcl_tour_costs[i] for i in 1: data.dim_depots],
        "final_dwell_times"=>[[final_sol.vhcl_dwell_times[i][t] 
                                    for t in final_sol.vhcl_tours[i][2:length(final_sol.vhcl_tours[i])-1]] 
                                for i in 1: data.dim_depots],
        "final_obj"=>final_sol.soln_cost,
        "final_time"=>solver.perturbation_time
    )
end

@class Solver begin
    start_time::BigFloat # Start time of the solver
    progress::Dict # Progress made by the heuristic
    final_solution::AbstractRoute # Final solution obtained from the heuristic
    instance::AbstractInstance # Instance object
    time_limit::Float16  # Time limit imposed on the heuristic
    initial_solution::AbstractRoute # initial solution
    local_search_solution::AbstractRoute # local search solution
    initial_time::Float64 # initial objective time
    local_search_time::Float64 # local search time
    perturbation_time::Float64 # final time after perturbation

    # Obtaining the solution using VNS.
    # Here, construction_mode depicts the method in which the initial solution is
    # generated.
    # mode_shake_phase denotes the neighborhoods considered for shaking
    # mode_local_search denotes the sequence of neighborhoods considered for local search
    # local_search_param denotes specific parameters for each neighborhood. For example,
    # this could denote the number of vehicles considered in a one-point move neighborhood.
    # time_limit is the maximum time for the heuristic.
    function Solver(instance::AbstractInstance; construction_mode = "min_sum",
         mode_shake_phase = "p", mode_local_search = "1", 
         local_search_param_dicts = [Dict("num_vhcls_switch" => 2, "vhcl_removal_choices" => ["longest tour","most targets"])], 
         time_limit = 7200.0,debug=false)

        start_time = time()
        progress = Dict()
        best_found = NaN
        initial_solution=NaN
        local_search_solution=NaN

        initial_time=0.
        local_search_time=0.
        perturbation_time=0.
        
        # Obtaining the neighborhoods for local search
        local_search_moves = []
        local_search_params = []
        for i in eachindex(mode_local_search)
            if cmp(mode_local_search[i], '1')==0 # Checking if a one-opt move is considered
                push!(local_search_moves,one_pt_move) # TO BE DEFINED
                push!(local_search_params,local_search_param_dicts[i])
            elseif cmp(mode_local_search[i], '2')==0 # Checking if a two-opt move is considered
                push!(local_search_moves,two_pt_move) # TO BE DEFINED
                push!(local_search_params,local_search_param_dicts[i])
            end

        end

        # Obtaining the shaking steps
        shaking_phase = []
        for i in mode_shake_phase
            if cmp(i, 'p')==0 # Checking if a perturbation step is considered
                push!(shaking_phase,perturbation) # TO BE DEFINED
            end
        end
        if debug
            println("constructing initial solution")
        end
        # Constructing an initial solution
        x = construct_solution(instance, construction_mode)
        best_found = route_copy(x)
        if debug
            println("INITIAL tour costs:\t",best_found.vhcl_tour_costs)
        end
        

        # Updating the best solution found, and updating the progress
        initial_time=time() - start_time
        progress[initial_time] = best_found.soln_cost
        initial_solution=route_copy(best_found)

        # we can end here if there is only one vehicle
        if instance.dim_depots==1
            return new(start_time, progress, x, instance, time_limit,
                        initial_solution,initial_solution, # copy initial solution (also x) for all solutions
                        initial_time,local_search_time,perturbation_time)
        end
        if debug
            println("Constructed initial solution")#, validity: ",check_if_valid(x))
            println("starting initial local search")
        end

        # Performing a local search on the initial solution
        j, j_max = 1, length(local_search_moves)+1
        while j < j_max
            x__ = local_search_moves[j](x, local_search_params[j])
            #if debug
                #println("local search j: ",j,", validity: ",check_if_valid(x__))
            #end
            # Checking if the current solution is strictly better than
            # previous incumbent solution
            if x__.soln_cost > x.soln_cost
                x = x__
                best_found = route_copy(x)
                if debug
                    println("local search found, objective: ",best_found.soln_cost)
                end
                j = 1 # Resetting the counter
            else # Moving to a different neighborhood
                j += 1
            end
        end
        if debug
            println("LOCAL SRCH tour costs:\t",best_found.vhcl_tour_costs)
            println("finished initial local search")#, validity: ",check_if_valid(x))
            println("starting VNS heuristic")
        end
        # Updating the progress
        local_search_time=time() - start_time
        progress[local_search_time] = best_found.soln_cost
        local_search_solution=route_copy(best_found)

        # Implementing the VNS heuristic on top of the obtained incumbent solution
        k, k_max = 0, 5 # Setting counters for shaking
        # initialize out here to avoid scope errors
        angle_perturb_depots=NaN
        avg_distance=NaN
        while k < k_max

            # Shaking phase. Here, we keep track of angle of first perturbation
            # for the "perturbation" step in shaking, which is one of the neighbourhoods
            # considered for perturbation. In addition, the average distance of
            # perturbation of depots is also kept track of for the same reason.

            if k == 0
                angle_perturb_depots = []
                avg_distance = []
            end

            x_, angle_perturb_depots, avg_distance = shaking(x, k, local_search_moves,
             local_search_params, shaking_phase, angle_perturb_depots, avg_distance)

            # Checking if time limit is reached for the heuristic
            if time() - start_time >= time_limit
                progress[time_limit] = best_found.soln_cost
                return new(start_time, progress, best_found, instance, time_limit,
                        initial_solution,local_search_solution,
                        initial_time,local_search_time,perturbation_time)
            end

            # Performing a local search on the solution obtained after shaking
            #if debug
            #    println("Shaken, performing local search.")
            #end
            x__=NaN#route_copy(x_)# initialize x__ outside to avoid scope errors
            j, j_max = 1, length(local_search_moves)+1
            while j < j_max
                x__ = local_search_moves[j](x_, local_search_params[j])
                #if debug
                #    println("j: ",j,"validity: ",check_if_valid(x__))
                #end
                # Checking if the current solution is strictly better than previous
                # incumbent solution
                if x__.soln_cost > x_.soln_cost
                    x_ = x__
                    j = 1
                else # Moving to next neighborhood for descent
                    j += 1
                end
            end

            # Checking if the obtained solution is better than the incumbent solution
            if x__.soln_cost > x.soln_cost
                x = x__
                k = 0 # Resetting the shaking counter
                best_found = route_copy(x)
                if debug
                    println("A better solution is obtained.")
                    println("value: ",best_found.soln_cost)
                end
                progress[time() - start_time] = best_found.soln_cost
            else
                k += 1
            end

            # Checking if time limit is reach
            if time() - start_time >= time_limit
                progress[time_limit] = best_found.soln_cost
                return new(start_time, progress, best_found, instance, time_limit,
                        initial_solution,local_search_solution,
                        initial_time,local_search_time,perturbation_time)
            end

        end

        # Updating the final progress
        perturbation_time=time() - start_time
        progress[perturbation_time] = best_found.soln_cost
        
        if debug
            println("FINAL tour costs:\t",best_found.vhcl_tour_costs)
        end
        return new(start_time, progress, best_found, instance, time_limit,
        initial_solution,local_search_solution,
        initial_time,local_search_time,perturbation_time)

    end

end