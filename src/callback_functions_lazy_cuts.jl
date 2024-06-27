using Graphs, Combinatorics, GraphsFlows, SparseArrays

function cuts_LP_lazycallback()

    function subtour_elimination_LP_cut(cb_data)

        # println(abs.(callback_value.(Ref(cb_data), model[:x])))
        status = callback_node_status(cb_data, model)
        # println("Status is ", status)
        # println(model)
        # Checking if the obtained solution is fractional
        if status == MOI.CALLBACK_NODE_STATUS_FRACTIONAL

            # Obtaining the values of the x variables
            x_vals = (abs.(callback_value.(Ref(cb_data), model[:x])))
            # println(x_vals)
            
            # Constructing a graph and adding flows from one target to another
            # using the x-values. NOTE: To account for numerical issues, the
            # edge weights are rounded to integers using a precision parameter.
            prec_val = 6.0

            flow_edges = []
            # Constructing a new array of vertices, starting from 1 to len(T). This
            # is used to account for issues in node numbering.
            T_new = [i for i in range(1, length(T))]

            for e in combinations(T_new, 2)

                # Checking if the x-value is non-zero
                if x_vals[min(T[e[1]], T[e[2]]), max(T[e[1]], T[e[2]])] >= 10^(-prec_val)

                    # Adding edge to the list. Note that flow in both directions is
                    # added
                    push!(flow_edges, (e[1], e[2], ceil(x_vals[min(T[e[1]], T[e[2]]), max(T[e[1]], T[e[2]])]*10^prec_val)))
                    push!(flow_edges, (e[2], e[1], ceil(x_vals[min(T[e[1]], T[e[2]]), max(T[e[1]], T[e[2]])]*10^prec_val)))
                    # append!(list_sources, e[1])
                    # append!(list_destinations, e[2])
                    # append!(list_edge_weights, ceil(x_vals[e[1], e[2]]*10^prec_val))

                end

            end
            # Constructing the graph
            # println(list_edge_weights)
            # G_flow = SimpleWeightedGraph(list_sources, list_destinations, list_edge_weights)

            flow_graph = Graphs.DiGraph(length(T))
            capacity_matrix = zeros(Int, length(T), length(T))  # Create a capacity matrix
            # Constructing the capacity matrix
            for e in flow_edges
                u, v, f = e
                Graphs.add_edge!(flow_graph, u, v)
                capacity_matrix[u,v] = f
            end
            # println("Vertices in flow graph is ", vertices(flow_graph))

            # Fixing node 1 in T_new as the "depot", and checking flow from every target
            # to the depot
            # Keeping track of the first partition obtained from the flow constraints.
            # Keeping track of this ensures that we don't add some flow constraint
            # multiple times.
            partition_1 = []
            for t in T_new
                if t == 1

                    continue
                    
                else

                    # println("Checking flow for target ", t)
                    # Checking flow from the target to node 1, the depot. Note that
                    # the flow value will be scaled by 10^precision
                    f, flow_matrix = maximum_flow(flow_graph, 1, t, capacity_matrix)
                    
                    if f <= 2 - 10^(-prec_val + 1) # Taking a marginally larger
                        # tolerance

                        # println("Flow from target ", t, " is ", f)
                        # Obtaining the partition (code was used from GitHub implementation)
                        residual_matrix = spzeros(Graphs.nv(flow_graph),Graphs.nv(flow_graph))
                        for edge in Graphs.edges(flow_graph)
                            residual_matrix[edge.src,edge.dst] = max(0.0, capacity_matrix[edge.src,edge.dst] - flow_matrix[edge.src,edge.dst])
                            residual_matrix[edge.dst,edge.src] = max(0.0, capacity_matrix[edge.dst,edge.src] - flow_matrix[edge.dst,edge.src])
                        end
                        part1 = typeof(1)[]
                        queue = [1]
                        while !isempty(queue)
                            node = pop!(queue)
                            push!(part1, node)
                            dests = [dst for dst in 1:Graphs.nv(flow_graph) if residual_matrix[node,dst]>0.0 && dst ∉ part1 && dst ∉ queue]
                            append!(queue, dests)
                        end
                        part2 = [node for node in 1:Graphs.nv(flow_graph) if node ∉ part1]

                        # Sorting partition 1
                        part1 = sort(part1)

                        # Checking if constraint was already imposed
                        # println("LHS of constraint obtained is ", cons)
                        if part1 in partition_1

                            continue

                        else

                            # Obtaining constraint corresponding to edges that go across the cut-set
                            cons = sum([model[:x][min(T[i], T[j]), max(T[i], T[j])] for i in part1 for j in part2])
                            flow_cons =  @build_constraint(cons >= 2)
                            # println("Building the flow constraint.")
                            # println("Adding flow constraint ", flow_cons)
                            MOI.submit(model, MOI.UserCut(cb_data), flow_cons)
                            # Appending the partition
                            # println(part1, part2, partition_1)
                            push!(partition_1, part1)

                        end

                    end

                end
            end

        end

    end

end

function constraint_lazycallback()

    function subtour_elimination_callback(cb_data)

        # status = callback_node_status(cb_data, model)
        # if status == MOI.CALLBACK_NODE_STATUS_FRACTIONAL
            # `callback_value(cb_data, x)` is not integer (to some tolerance).
            # If, for example, your lazy constraint generator requires an
            # integer-feasible primal solution, you can add a `return` here.
            # return
        # elseif status == MOI.CALLBACK_NODE_STATUS_INTEGER
        # print(cb_data)

        # println(abs.(callback_value.(Ref(cb_data), x)))
        status = callback_node_status(cb_data, model)
        # println("Status is ", status)
        # println(model)

        # Checking if the obtained solution is integral
        if status == MOI.CALLBACK_NODE_STATUS_INTEGER

            # Obtaining the values of the x variables
            x_vals = (abs.(callback_value.(Ref(cb_data), model[:x])))
            # println(x_vals)

            # Constructing a new array of vertices, starting from 1 to len(T). This
            # is used to account for issues in node numbering.
            T_new = [i for i in range(1, length(T))] 

            # Constructing a graph and adding edges for which the x_val is non-zero
            edges_traversed = Edge.([(e[1], e[2]) for e in combinations(T_new, 2)
             if x_vals[min(T[e[1]], T[e[2]]), max(T[e[1]], T[e[2]])] > 0.5])
            G = Graphs.SimpleGraph(edges_traversed)
            # println(edges_traversed)

            # Obtaining the connected components in the graph
            conn_comps = connected_components(G)

            # # Removing those vertices from conn_comps that are not in T. NOTE:
            # # This is an issue with SimpleGraph in Julia, wherein node numbers are
            # # set from 1 to n, where n is the number of nodes
            # conn_comps_filtered = []
            # for conn_comp in conn_comps
            #     if length(conn_comp) == 1
            #         if conn_comp[1] ∉ T
            #             continue
            #         end
            #     end
            #     push!(conn_comps_filtered, conn_comp)
            # end

            # println(conn_comps, "Filtered - ", conn_comps_filtered, length(conn_comps_filtered))

            # Checking if there is more than one connected component in the graph
            if length(conn_comps) > 1
                
                # Adding constraints connecting each connected component with another
                # Running through each connected component
                for conn_comp in conn_comps

                    # Obtaining the list of edges that are in the cutset
                    # println(conn_comp)
                    vertices_outside = []
                    for j in T_new
                        if j in conn_comp
                            continue
                        else
                            append!(vertices_outside, j)
                        end
                    end
                    # println("Vertices inside and outside are", conn_comp, vertices_outside)
                    cons = sum([model[:x][min(T[i], T[j]), max(T[i], T[j])] for i in conn_comp for j in vertices_outside])
                    connectivity_cons = @build_constraint(cons >= 2)
                    MOI.submit(model, MOI.LazyConstraint(cb_data), connectivity_cons)

                end
                
            end

        end

    end
end