# plots vehicle objective value wrt number of targets
# shows how the objective obtained by vehicle varies with number of targets
# calls target_number_experiment.py

include(joinpath("src","route.jl"))
include(joinpath("src","construct.jl"))
include(joinpath("src","highs_single_vehicle_TSP_dwelltime_functions.jl"))

PYTHON_COMMAND="python"
PYTHON_FILE="target_number_experiment.py"

alpha_factor=1.
tau=20.

temp_dir=joinpath("output","temp")
output_dir=joinpath("output","plots")

temp_file=joinpath(temp_dir,"targets_alpha_"*string(alpha_factor)*"_tau_"*string(tau)*".txt")
output_file=joinpath(output_dir,"targets_alpha_"*string(alpha_factor)*"_tau_"*string(tau)*".png")
cleanup=false

for d in (temp_dir,output_dir)
    if !isdir(d)
        mkpath(d)
    end
end

# check if same trend is reflected by wide range of alpha/tau variables
# PARAMETRIC STUDY pick diverse MMs (number of targets per vehicle)
# first change neighborhood to picking highest objective value and remove it
# 
test_alpha=1.

md_folder=joinpath("input_data","MD_algorithm_datasets")
data=Data("MM22",md_folder)
test_instance=Instance(data,test_alpha,1.)
test_tour=construct_tour_LKH(test_instance,collect(keys(test_instance.targets)))
test_tour_cost=path_cost(test_instance,test_tour)
# alpha is 1/tsp cost


f=open(temp_file,"w")
write(f,"[")
println("TEST TOUR COST: ",test_tour_cost)
alpha=alpha_factor/test_tour_cost
println("ALPHA: ",alpha)
instance=Instance(data,alpha,tau)
for i in 1:instance.dim_targets
    start_time = time()
    tour, dwell_times, tour_cost, key_array=single_vehicle_TSP_dwell_times(instance,[j for j in 1:i];tolerance=.0001)
    G=undiscounted_info_gain_func(dwell_times,[instance.tau for _ in key_array])
    obj_fun=exp(-instance.alpha*sum(dwell_times))*exp(-instance.alpha*tour_cost)*G
    println(i," targets: ",obj_fun)
    write(f,string(obj_fun))
    write(f,",")
    println("time: ",time()-start_time)
end
write(f,"]")
close(f)
run(`$PYTHON_COMMAND $PYTHON_FILE $temp_file $output_file $cleanup`)