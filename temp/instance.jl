using Classes
include("data.jl")

# In this function, an Instance object is constructed corresponding to the
# considered graph

@class Instance begin
    targets::Dict
    depots::Dict
    cost_traversal::Dict
    data::AbstractData
    alpha::Float16
    tau::Float16

    # Writing a construction function
    function Instance(data::AbstractData)

        # Obtaining the cost of traversal from one target to another, and
        # from a target to a depot
        cost_traversal = Dict()

        for pois in (data.targets,data.depots)
            # running through each 'poi', either target or depots
            for i in keys(pois)
                # Checking if key "i" is present in the dictionary. If not, key
                # is created
                if haskey(cost_traversal, i)
                    continue
                else
                    cost_traversal[i] = Dict()
                end

                # Running through targets/depots
                for pois2 in (data.targets,data.depots)
                    for j in keys(pois2)
                        if i != j # Checking if the same target is not considered
                            # Computing the distance of traversal from i to j
                            cost_traversal[i][j] = sqrt((pois[i][1] - pois2[j][1])^2
                            + (pois[i][2] - pois2[j][2])^2)
                        end
                    end
                end
            end
        end

        return new(data.targets, data.depots, cost_traversal, data, 0, 0)

    end

end