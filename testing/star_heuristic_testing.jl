# testing the star heuristic function
using JuMP
using Graphs
using HiGHS 
#using Gurobi
import MathOptInterface as MOI
include(joinpath("..", "src","route.jl"))
# include(joinpath("..", "src","highs_single_vehicle_TSP_dwelltime_functions.jl"))
include(joinpath("..", "src","construct.jl"))

md_folder=joinpath("input_data","MD_algorithm_datasets")
d=Data("MM7",md_folder)
instance=Instance(d,1.,2.)
i2=_perturb_similar_depots(instance)
#model=partition_star_heuristic_model(i2)
#optimize!(model)
route=partition_tours_star_heuristic(instance)
#for depot_key in keys(instance.depots)
#    println("DEPOT ",depot_key," ASSIGNED TO:")
#    for target_key in keys(instance.targets)
#        if value(model[:x][target_key,depot_key])==1.0
#            println("\t",target_key)
#        end
#    end
#end