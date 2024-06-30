# uses the HiGHS optimizer for solving single vehicle routing problem

# In this function, a single vehicle TSP is solved using exact algorithm or
# using LKH.

using JuMP, HiGHS, LinearAlgebra, Random, Plots
include("instance.jl")
include("callback_functions_lazy_cuts.jl")
include("running_LKH.jl")
include("grad_descent.jl")

#constraint_lazycallback()
#cuts_LP_lazycallback()

import MathOptInterface as MOI

function single_vehicle_TSP_model(instance::AbstractInstance, vertices)
    # Here, instance corresponds to the instance object corresponding to the
    # considered graph. Further, vertices represents the array of vertices covered 
    # by vehicle number "vhcl_no". It should be noted that this includes the depot
    # of the vehicle as well.

    # Initializing the model (without the subtour constraints) 
    #model = Model(Gurobi.Optimizer)
    model = Model(HiGHS.Optimizer)
    MOI.set(model, MOI.Silent(), true)

    # Defining binary variables for the edges
    @variable(model, x[i in vertices, j in vertices; i < j], Bin)

    # Adding the degree constraint
    @constraint(model, c[j in vertices], sum(x[i, j] for i in vertices if i < j)
     + sum(x[j, i] for i in vertices if i > j) == 2)

    # Adding the objective function
    @objective(model, Min, sum(instance.cost_traversal[min(i, j)][max(i, j)]*x[min(i, j), max(i, j)]
     for (i, j) in combinations(vertices, 2)))

    # print(model)
    return model

end

function single_vehicle_TSP_dwell_times(instance::AbstractInstance, 
                                                targ_covered, 
                                                vhcl_no=1,
                                                exact=true;
                                                beta1=.2,
                                                beta2=.5,
                                                tolerance=.0001,
                                                initial=nothing
                                                )
    # solves the TSP problem and the dwell times for a single vehicle
    # uses single_vehicle_TSP_exact to solve the tour
    # assumes targ_covered is an array of POI keys 
    #       also that targ_covered does not include depots
    # exact is a boolean, if true, we find exact tour 
    #   if false, we just use LKH
    # beta1 and beta2 are used for gradient descent
    # returns tour, dwell times, tour cost, and key_array
    #       key_array should be redundant, it is returned since targ_covered is changed in single_vehicle_TSP_exact
    #       dwell times are in the same order as targ_covered, and in same order as the return key_array
    key_array = [key for key in targ_covered]
    taui_arr = [instance.tau for key in key_array]
    if exact
        tour,tour_cost = single_vehicle_TSP_exact(instance,targ_covered,vhcl_no)
    else
        tour=single_vehicle_TSP_LKH(instance,targ_covered,vhcl_no)
        tour_cost=path_cost(instance,tour)
    end

    dwell_times=nesterov_gradient_descent(taui_arr,instance.alpha,1.,initial=initial,tolerance=tolerance)

    return tour, dwell_times, tour_cost, key_array
end

function path_cost(instance,tour)
    # returns path cost of tour, assumes it starts and ends with same node
    # path_cost(Instance, [1,2,3,1])
    path_cost=0
    for i in range(1,length(tour)-1)
        path_cost+=instance.cost_traversal[tour[i]][tour[i+1]]
    end
    return path_cost
end
function single_vehicle_TSP_LKH(instance::AbstractInstance, targ_covered, vhcl_no = 1)
    # returns tour obtained from LKH
    insert!(targ_covered, 1, collect(keys(instance.depots))[vhcl_no])
    loop=construct_tour_LKH(instance,targ_covered)
    if length(loop)<=3
        return loop
    end

    edges_tour = [(min(loop[i], loop[i + 1]), max(loop[i], loop[i + 1])) for i in 1: length(loop) - 1]
    edges_traversed = Edge.(edges_tour)
    G_soln = Graphs.SimpleGraph(edges_traversed)
    # Obtaining the cycle starting at the depot
    cycle = cycle_basis(G_soln, collect(keys(instance.depots))[vhcl_no])[1]
    # Inserting node 1 in the beginning, since the rest of the cycle would have been returned
    insert!(cycle, 1, collect(keys(instance.depots))[vhcl_no])
    return cycle    
end
function single_vehicle_TSP_exact(instance::AbstractInstance, targ_covered, vhcl_no = 1)
    # In this function, the integer program formulation for the single vehicle
    # TSP is solved using lazy cuts.

    global model
    global T
    # Obtaining new set of vertices to be covered including the depot of the considered
    # vehicle
    insert!(targ_covered, 1, collect(keys(instance.depots))[vhcl_no])
    if length(targ_covered)<4
        push!(targ_covered,collect(keys(instance.depots))[vhcl_no])
        return targ_covered,path_cost(instance,targ_covered)
    end
    T = targ_covered
    # println(T)

    # Obtaining the model containing the integer program formulation
    model = single_vehicle_TSP_model(instance, T)
    # print(model)

    # Warm starting the model using LKH
    # Obtaining the tour from LKH
    tour = construct_tour_LKH(instance, T)
    # println(tour)
    # Obtaining the list of edges in the tour
    edges_tour = [(min(tour[i], tour[i + 1]), max(tour[i], tour[i + 1])) for i in 1: length(tour) - 1]
    # println(edges_tour)
    # Running through all edges in the model
    for e in combinations(T, 2)
        if (min(e[1], e[2]), max(e[1], e[2])) in edges_tour
            set_start_value(model[:x][min(e[1], e[2]), max(e[1], e[2])], 1)
            # println("Setting ", [min(e[1], e[2]), max(e[1], e[2])], " to 1")
        else
            set_start_value(model[:x][min(e[1], e[2]), max(e[1], e[2])], 0)
        end
    end

    # Adding lazy cuts
    #MOI.set(model, MOI.RawOptimizerAttribute("LazyConstraints"), 1)
    #set_attribute(model, MOI.LazyConstraintCallback(), constraint_lazycallback())
    #set_attribute(model, MOI.UserCutCallback(), cuts_LP_lazycallback())
    optimize!(model)

    # Retrieving the solution
    # Reading the optimal solution and returning the tour
    # Running through all variables in the model to obtain the tour
    arr_edges_traversed = [(e[1], e[2]) for e in combinations(T, 2)
     if value(model[:x][min(e[1], e[2]), max(e[1], e[2])]) > 0.5]
    edges_traversed = Edge.(arr_edges_traversed)
    # println(edges_traversed)
    G_soln = Graphs.SimpleGraph(edges_traversed)
    # Obtaining the cycle starting at the depot
    cycle = cycle_basis(G_soln, collect(keys(instance.depots))[vhcl_no])[1]
    # Inserting node 1 in the beginning, since the rest of the cycle would have been returned
    insert!(cycle, 1, collect(keys(instance.depots))[vhcl_no])

    return cycle, objective_value(model)

end
