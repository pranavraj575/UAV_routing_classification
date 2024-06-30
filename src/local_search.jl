include("single_vehicle_TSP_dwelltime_functions.jl")
include("running_LKH.jl")


function one_point_best_proxy_dict(instance,partition,tau_dict,alpha,dwell_times_dict,tours,constants=[])
    # finds best neighbor from a one point move
    # uses proxy costs, assumes dwell times are same,
    #     and tours are changed by simple insertion/deletion
    Jis=obj_function_by_vehicle(instance,partition,tau_dict,alpha)
    neighbor_to_proxy_improvement=Dict()
    Gis=[]
    for part in partition
        dwell_arr=[dwell_times_dict[key] for key in part]
        taui_arr=[tau_dict[key] for key in part]
        push!(Gis,undiscounted_info_gain_func(dwell_arr,taui_arr))
    end
    for a in range(1,length(partition))
        # moving from partiton of vehicle A, index a
        JA=Jis[a]
        GA=Gis[a]
        tourA=tours[a]
        assert(length(tourA)>1)
        for b in range(1,length(partition))
            # moving to parititon of vehicle B,index b
            JB=Jis[b]
            GB=Gis[b]
            tourB=tours[b]
            assert(length(tourB)>1)
            # moving to parition[j]
            if !(a==b)
                # if they are not the same
                for (kA,keyA) in enumerate(tourA)
                    # which element we are planning to move
                    if !(kA in constants)
                        neighbor=deepcopy(partition)
                        del_element(neighbor[a],keyA)
                        push!(neighbor[b],keyA)

                        best_prox_diff=-Inf
                        

                        # initially tour A looks like
                        #       keyAp => keyA => keyppA
                        # changing to 
                        #       keyAp => keyppA
                        if kA==length(tourA)
                            # we have already done this with first element
                            continue
                        elseif kA==1
                            keyAp=tourA[length(tourA)]
                            keyApp=tourA[kA+1]
                        else
                            keyAp=tourA[kA-1]
                            keyApp=tourA[kA+1]
                        end
                        # this factor changes the path cost to the correct proxy path
                        factor_1A=exp(-alpha*(
                            -instance.cost_traversal[keyAp][keyA] # remove
                            -instance.cost_traversal[keyA][keyApp] # remove
                            +instance.cost_traversal[keyAp][keyApp] # add
                        ))
                        # this is removing the dwell time from the discount term

                        # dA is dwell time of the element we are removing from vehicle A
                        # tauA is tau of the element we remove from vehicle A
                        dA=dwell_times_dict[keyA]
                        tauA=tau_dict[keyA]
                        factor_2A=exp(alpha*dA)

                        # this corrects the info gain term GA
                        factor_3A=(GA-Ii(dA,tauA))/GA

                        # proxy JA cost
                        proxy_JA=JA*factor_1A*factor_2A*factor_3A

                        for (kB,keyB) in enumerate(tourB)
                            if kB==length(tourB)
                                # we have already considered this with first element
                                continue                                
                            end

                            keyBp=tourB[kB+1]
                            # initially tour B looks like
                            #       keyB => keyBp
                            # changing to 
                            #       keyB => keyA => keyBp

                            
                            # this factor changes the path cost to the correct proxy path
                            factor_1B=exp(-alpha*(
                                +instance.cost_traversal[keyB][keyA] # add
                                +instance.cost_traversal[keyA][keyBp] # add
                                -instance.cost_traversal[keyB][keyBp] # remove
                            ))
                            
                            # this is adding the dwell time to the discount term

                            # dA is dwell time of the element we are adding to vehicle B
                            # tauA is tau of the element we add to vehicle B
                            factor_2B=exp(-alpha*dA)
                            # this corrects the info gain term GA
                            factor_3B=(GB+Ii(dA,tauA))/GB

                            # proxy JB cost
                            proxy_JB=JB*factor_1B*factor_2B*factor_3B

                            proxy_difference=proxy_JA+proxy_JB-(JA+JB)
                            best_prox_diff=max(best_prox_diff,proxy_difference)
                        end
                        push!(neighbor_to_proxy_improvement,neighbor=>best_prox_diff)
                    end
                end
            end
        end
    end
    return neighbors_to_proxy_improvement
end

function one_point_neighbors(partition,constants=[])
    # move a single point from one set to another
    # constants are elements that should not be moved
    neighbors=[]
    for (i,part) in enumerate(partition)
        # moving from part
        for j in range(1,length(partition))
            # moving to parition[j]
            if !(i==j)
                # if they are not the same
                for (k,pt) in enumerate(part)
                    if !(pt in constants)
                        # remove a non-constant pt
                        neigh=deepcopy(partition)
                        deleteat!(neigh[i],j)
                        push!(neigh[k],pt)

                        push!(neighbors,neigh)
                    end
                end
            end
        end
    end
    return neighbors
end

function del_element(vector,element)
    # deletes element from vector, returns result
    deleteat!(vector, findall(x->x==element,vector))
    return vector
end

function lazy_obj_function_by_vehicle(instance,partition,tau_dict,alpha,dwell_times_dict,tours)
    # returns objective function for given partition for each vehicle
    # assumes dwell times are constant
    # partition is a vector of key vectors
    #   i.e. [[1,2],[3,4]] corresponds to assigning targets 1 and 2 to vehicle 1 and 3 and 4 to vehicle 2
    # the keys should both be able to access tau_dict and the Instance
    # gets a tour for each vehicle
    Jis=[]

    for (p,keys) in enumerate(partition)
        tau_vector=[tau_dict[key] for key in keys]
        dwell_times=[dwell_times_dict[key] for key in keys]
        #neg_log=dwell_time_info_gain_func(dwell_times,tau_vector,alpha)
        
        tour=tours[p]
        path_cost=path_cost(instance,tour)
        Ji=exp(-alpha*(sum(dwell_time_arr)))*exp(-alpha*path_cost)*undiscounted_info_gain_func(dwell_times,tau_vector)
        push!(Jis,Ji)
    end
    return Jis
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

function obj_function_by_vehicle(instance,partition,tau_dict,alpha)
    # returns objective function for given partition for each vehicle
    # runs gradient descent to find dwell times
    # uses LKH right now for tours, since cannot use Gurobi
    # partition is a vector of key vectors
    #   i.e. [[1,2],[3,4]] corresponds to assigning targets 1 and 2 to vehicle 1 and 3 and 4 to vehicle 2
    # the keys should both be able to access tau_dict and the Instance
    tours=[]
    dwell_times_dict=Dict()
    for keys in partition
        tau_vector=[tau_dict[key] for key in keys]
        dwell_times=gradient_descent(tau_vector,alpha,.2,.5)
        for (i,d) in enumerate(dwell_times)
            push!(dwell_times_dict,keys[i]=>d)
        end
        
        tour=construct_tour_LKH(instance,keys)
        push!(tours,tour)
    end
    return lazy_obj_function_by_vehicle(instance,partition,tau_dict,alpha,dwell_times_dict,tours)
end
function obj_function(instance,partition,tau_dict,alpha)
    # returns objective function for given partition
    # runs gradient descent to find dwell times
    # uses LKH right now for tours, since cannot use Gurobi
    # partition is a vector of key vectors
    #   i.e. [[1,2],[3,4]] corresponds to assigning targets 1 and 2 to vehicle 1 and 3 and 4 to vehicle 2
    # the keys should both be able to access tau_dict and the Instance
    return sum(obj_function_by_vehicle(instance,partition,tau_dict,alpha))
end