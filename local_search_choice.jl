include("vns.jl")
include("utils.jl")

PYTHON_COMMAND="python"
PYTHON_FILE="local_search_choice.py"
param=2
local_search_mode="12"
temp_file="temp//bar_neighborhoods_"*local_search_mode*"_param"*string(param)*".txt"
output_file="plots//bar_neighborhoods_"*local_search_mode*"_param"*string(param)*".png"
table_file="data_files//improvement_table_neighborhoods_"*local_search_mode*"_param"*string(param)*".tex"
cleanup=false
f=open(temp_file,"w")
write(f,"{")

folder="MD algorithm datasets"

prefixes=[]
files=readdir(folder)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end


# prefixes=["MM17"]
# prefixes=["MM1"]
#prefixes=["MM"*string(i) for i in 1:10]

prefixes=sort(prefixes,by=number_from_file)
folder='\\'*folder
for file in prefixes
    write(f,"'"*file*"'")
    write(f,":{")
    println("\n\n",file,"\t",folder)
    data=Data(file,folder)
    tau=2.
    # figuring out alpha
    test_alpha=1.
    # alpha is 1/tsp cost
    test_instance=Instance(data,test_alpha,tau)
    test_tour=construct_tour_LKH(test_instance,collect(keys(test_instance.targets)))
    test_tour_cost=path_cost(test_instance,test_tour)    
    println("TEST TOUR COST: ",test_tour_cost)
    alpha=1/test_tour_cost
    println("ALPHA: ",alpha)

    instance=Instance(data,alpha,tau)
    initial=[]
    for removal_types in [["smallest obj"],
                            ["largest obj"],
                            ["most targets"], 
                            ["longest tour"],
                            ["longest tour", "most targets"]]
        #println("TEST: ",instance.cost_traversal[][1])
        removal_type=NaN
        if length(removal_types)==1
            removal_type=removal_types[1]
        elseif removal_types==["longest tour", "most targets"]
            removal_type="combo"
        else
            removal_type="combo2"
        end
        solver=Solver(instance,mode_local_search = local_search_mode,
                        local_search_param_dicts=[Dict("num_vhcls_switch" => param, "vhcl_removal_choices" => removal_types) for _ in 1:length(local_search_mode)])
        
        solver.local_search_solution.soln_cost
        push!(initial,solver.initial_solution.soln_cost)
        write(f,"'"*removal_type*"':")
        write(f,string(solver.local_search_solution.soln_cost))
        write(f,",")
    end
    
    write(f,"'initial':")
    write(f,string(sum(initial)/length(initial)))
    write(f,",")
    write(f,"},")
end

write(f,"}")
close(f)
run(`$PYTHON_COMMAND $PYTHON_FILE $temp_file $output_file $table_file $cleanup`)