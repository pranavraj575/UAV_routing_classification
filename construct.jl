using JuMP
using Graphs
using HiGHS 
#using Gurobi
import MathOptInterface as MOI
include("route.jl")
include("highs_single_vehicle_TSP_dwelltime_functions.jl")

function partition_tours_star_heuristic(instance::AbstractInstance,exact=false)
    # fully constructs a route object
    # partitions instance according to the star heuristic
    # exact is whether to exactly solve the tours
    #   if false, uses just LKH
    # creates tours for each partition according to single_vehicle_TSP_exact
    # uses gradient descent to solve for dwell times
    # returns a Route object, contains tours and other stuff in it


    # only using instance2 for the initial parition, everything else uses original instance
    instance2=_perturb_similar_depots(instance)
    model=_partition_star_heuristic_model(instance2)
    optimize!(model)
    # create route
    R=Route(instance)
    for vhcl_no = 1:instance.data.dim_depots
        # this is the depot of the vehicle
        depot_key=collect(keys(instance.depots))[vhcl_no]

        # vertices that the depot should visit, according to solved model
        vertices=[v for v in keys(instance.targets) if value(model[:x][v,depot_key])==1.0]
        
        # correct number of taus
        tau_arr=[instance.tau for _ in vertices]

        # solves for tour and dwell times, as well as tour cost
        (tour, dwell_times, tour_cost, key_array)=single_vehicle_TSP_dwell_times(instance,vertices,vhcl_no,exact)
        neg_log=dwell_time_info_gain_func(dwell_times,tau_arr,instance.alpha)
        # this is G*e^(-alpha*(dwell times sum))*e^(-alpha*(TSP cost))

        objective_value=exp(-neg_log)*exp(-instance.alpha*tour_cost)

        # sets this
        route_set_vhcl_i_tour_tour_cost(R,tour,tour_cost,objective_value,vhcl_no)

        # sets dwell times
        for (i,key) in enumerate(key_array)
            R.vhcl_dwell_times[vhcl_no][key]=dwell_times[i]
        end
    end
    return R
end

function _partition_star_heuristic_model(instance::AbstractInstance)
    #model = Model(Gurobi.Optimizer)
    model = Model(HiGHS.Optimizer)
    MOI.set(model, MOI.Silent(), true)

    # we define variables using these keys
    # n and m are defined as the number of targets and depots respectively
    target_keys=keys(instance.targets)
    n=instance.data.dim_targets
    depot_keys=keys(instance.depots)
    m=instance.data.dim_depots
    
    # Defining binary variables for assignment of POIS to depots
    # x[i,j] represents assigning taget i to depot j
    # as used in the MD paper part 2.1
    @variable(model, x[i in target_keys, j in depot_keys], Bin)

    
    # Adding the constraint that every POI is assigned to exactly one depot
    # for each target i, sum(x[i,j] across all depots j)=1
    @constraint(model, c[i in target_keys], 
        sum(x[i, j] for j in depot_keys) == 1)
    
        
    # Adding the constraint that every depot is assigned to an equal number of targets
    extra=n%m # the number of depots that should have an extra target
    
    large_depots=[collect(depot_keys)[k] for k in range(1,extra)]
    small_depots=[collect(depot_keys)[k] for k in range(extra+1,m)]
    @constraint(model, l[j in large_depots], 
        sum(x[i, j] for i in target_keys) == ceil(n/m))

    @constraint(model, s[j in small_depots], 
        sum(x[i, j] for i in target_keys) == floor(n/m))

    #@constraint(model, l[j in depot_keys], 
    #sum(x[i, j] for i in target_keys) >= floor(n/m))

    #@constraint(model, h[j in depot_keys], 
    #sum(x[i, j] for i in target_keys) <= ceil(n/m))
    
    
    # Adding the objective function
    @objective(model, Min, 
                sum(
                    sum(
                        x[i,j]*instance.cost_traversal[i][j] for i in target_keys
                        ) 
                    for j in depot_keys
                    )
                )
    return model
end

function _perturb_similar_depots(instance::AbstractInstance,offset=.1,tolerance=.001)
    # returns a copy of instance such that the duplicated depots are perturbed 
    # section 3.1.1 of the MD paper
    # offset is the radius to perturb the duplicates
    # tolerance is the tolerance that depots count as duplicates

    # create an edge if two depots are too close
    edges=Edge.([(a,b) for (a,b) in combinations(collect(keys(instance.depots)),2)
        if instance.cost_traversal[a][b]<= tolerance])
    
    # if no edges, we can just return instance
    if length(edges)==0
        return instance
    end
    

    instance2=deepcopy(instance)
    g=SimpleGraph(edges)
    for conn_comp in connected_components(g)
        if length(conn_comp)>1
            com=_center_of_mass([instance2.depots[k] for k in conn_comp])
            rand_angle=rand()*pi*2
            for (i,key) in enumerate(conn_comp)
                angle=rand_angle+i*2*pi/length(conn_comp)
                # angle is distributed evenly, starting at random angle

                perturbed=(com[1]+offset*cos(angle),com[2]+offset*sin(angle))
                # perturbs the point around the com

                instance2.depots[key]=tuple(perturbed...,com[3:length(com)]...)
                # just in case dimensions are greater than 2
            end
        end
    end
    
    return instance2
end

function _center_of_mass(points)
    # returns center of mass of points
    # requires points to be nonempty
    # requires each point to have same DIMENSION
    return [sum(p[i] for p in points)/length(points) 
                for i in range(1,length(points[1]))]
end
