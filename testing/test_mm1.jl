include(joinpath("..", "src","route.jl"))
include(joinpath("..", "src","construct.jl"))
include(joinpath("..", "src","highs_single_vehicle_TSP_dwelltime_functions.jl"))
# check if same trend is reflected by wide range of alpha/tau variables
# PARAMETRIC STUDY pick diverse MMs (number of targets per vehicle)
# first change neighborhood to picking highest objective value and remove it
# 
test_alpha=1.
# alpha is 1/tsp cost
tau=2.
md_folder=joinpath("input_data","MD_algorithm_datasets")
data=Data("MM22", md_folder)
test_instance=Instance(data,test_alpha,tau)
test_tour=construct_tour_LKH(test_instance,collect(keys(test_instance.targets)))
test_tour_cost=path_cost(test_instance,test_tour)

println("TEST TOUR COST: ",test_tour_cost)
alpha=1/test_tour_cost
println("ALPHA: ",alpha)
instance=Instance(data,alpha,tau)
route=partition_tours_star_heuristic(instance)
println(route.soln_cost)

n=length(instance.targets)
target_keys=collect(keys(instance.targets))

m=length(instance.depots)

# dictionary of solved dwell times by number of targets
dwell_dict=Dict()

for i in 1:n
    taui=[instance.tau for _ in 1:i]
    dwell_dict[i]=gradient_descent(taui,instance.alpha,.2,.5)
end
for k in keys(sort(dwell_dict))
    println(k,": ",dwell_dict[k])
end

function m_ary_vector(number,m,length=n)
    # returns the m-ary representation of number
    # m=2 gives binary
    # length is the length of vector to output
    out=[]
    for i in 1:length
        pushfirst!(out,number%m)
        number=Int64(floor(number/m))
    end
    return out
end
best_partition=NaN
optim_obj_cost=-Inf
aa=-1
for part_number in 0:3^n-1
    print("PARTTITION: ",part_number+1,"/",3^n,'\r')
    global aa,optim_obj_cost,best_partition
    partition=m_ary_vector(part_number,m)
    obj_fun=0
    for vhcl_no in 1:m
        indices=findall(x->x+1==vhcl_no,partition)
        if length(indices)<1
            continue
        end
        targets=[target_keys[i] for i in indices]
        dwell_times=dwell_dict[length(targets)]
        tour=single_vehicle_TSP_LKH(instance,targets,vhcl_no)
        tour_cost=path_cost(instance,tour)
        
        G=undiscounted_info_gain_func(dwell_times,[instance.tau for _ in dwell_times])
        obj_fun+=exp(-instance.alpha*sum(dwell_times))*exp(-instance.alpha*tour_cost)*G
    end
    if obj_fun>optim_obj_cost
        println()
        println("better objective found: ",obj_fun)
        println("\tPartition:",partition)
        println()
        optim_obj_cost=obj_fun
        best_partition=partition
    end
end

println(best_partition)