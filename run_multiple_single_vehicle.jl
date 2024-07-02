# outdated method to write a table with dwell times and tour costs with single vehicle files
include(joinpath("src","instance.jl"))
include(joinpath("src","route.jl"))
include(joinpath("src","vns.jl"))
include(joinpath("src","single_vehicle_TSP_dwelltime_functions.jl"))
using Printf
output_file=joinpath("output","single_vhcl_results_table.tex")
if !isdir("output")
    mkdir("output")
end
function real_round(n,digits)
    return round(n*10^digits)/10^(digits)
end
function number_from_file(name)
    num=""
    for c in name
        if c in "0123456789"
            num=num*c
        end
    end
    return parse(Int32,num)
end

function write_cline(a=4,b=11)
   global f
   write(f,"\n\\cline{"*string(a)*"-"*string(b)*"}\n") 
end
function write_table_head()
    global f
    write(f,"\\begin{table*}[htb!]")
    write(f,'\n')
    write(f,"\\centering")
    write(f,'\n')
    write(f,"\\caption{Results for single vehicle case}\n")
    write(f,"\\label{tab: single_vehicle_results}")
    write(f,"\t\\begin{tabular}{|c|c|c|c|c|c|c|c|c|c|c|}")
    write(f,'\n')
    write(f,"\t\\hline")
    write(f,'\n')
    write(f,"\\multirow{2}{*}{\\textbf{Instance}} &")
    write(f,"\\textbf{Number} &")
    write(f,"\\multirow{2}{*}{\\textbf{TSP cost}} &")
    write(f,"\\multirow{2}{*}{\$\\alpha\$ (s\\textsuperscript{-1})} &")
    write(f,"\\multirow{2}{*}{\$\\tau\$ (s)} &")
    write(f,"\\multicolumn{4}{c|}{\\textbf{Dwell Time (s)}}&")

    write(f,"\\textbf{Branch and cut} &")
    write(f,"\\textbf{Gradient}\\\\")
    write_cline(6,9)


    write(f,"& {\\textbf{Targets}} & & & & \\textbf{Min}& \\textbf{Median}& \\textbf{Mean}& \\textbf{Max} & \\textbf{time} (s) & \\textbf{descent time} (s) \\\\")
    write(f,'\n')    
    write(f,'\n')
end
last_id=""
last_alpha=-1
function write_table_entry(identifier,number,tour_cost,alpha,tau,minn,mediann,menan,maxx, TSP_computation_time, dwell_time_comp_time)
    global f
    global last_id
    global last_alpha
    if last_id != identifier
        write(f,"\n\\hline\n ")
    elseif last_alpha!=alpha
        write_cline()
    end

    write(f,'\t')
    if identifier==last_id
        write(f,"& & &")
    else
        write(f,"\\multirow{9}{*}{"*string(identifier)*"}")
        write(f," & ")
        write(f,"\\multirow{9}{*}{"*string(number)*"}")
        write(f," & ")
        write(f,"\\multirow{9}{*}{"*string(real_round(tour_cost,2))*"}")
        write(f," & ")
    end
    if last_alpha==alpha
        write(f," & ")
    else
        write(f,"\\multirow{3}{*}{"*(@sprintf "%.2e" alpha)*"}")
        write(f," & ")
    end
    write(f,string(tau))
    write(f," & ")
    write(f,string(real_round(minn,2)))
    write(f," & ")
    write(f,string(real_round(mediann,2)))
    write(f," & ")
    write(f,string(real_round(menan,2)))
    write(f," & ")
    write(f,string(real_round(maxx,2)))
    write(f," & ")
    write(f,string(real_round(TSP_computation_time,2)))
    write(f," & ")
    write(f,(@sprintf "%.2e" dwell_time_comp_time))
    write(f," \\\\")
    write(f,'\n')
    last_id=identifier
    last_alpha=alpha
end
function write_table_tail()
    global f
    write(f,"\t\\hline")
    write(f,'\n')
    write(f,"\t\\end{tabular}")
    write(f,'\n')
    write(f,"\\end{table*}")
    write(f,'\n')
end
folder="tsp_files"

prefixes=[]
files=readdir(folder)
for file in files
    if occursin("_depots.tsp",file)
        prefix=file[1:length(file)-11]
        push!(prefixes,prefix)
    end
end
prefixes=sort(prefixes,by=number_from_file)
# prefixes=["MM17"]
# prefixes=["MM20"]
prefixes=["rd100","bier127","pr152",
"d198","pr226","gr229"]
prefixes=["rd100"]

insert!(prefixes,1,"rd100")
# println(prefixes)

f=open(output_file,"w")
write_table_head()

warmup=true

for file in prefixes
    global warmup
    println("\n\n",file,"\t",folder)
    data=Data(file,folder)
    current_tau=2.
    # figuring out alpha
    test_alpha=1.
    # alpha is 0.5/tsp_cost, 1/tsp cost, or 2/tsp cost
    test_instance=Instance(data,test_alpha,current_tau)
    all_vertices=collect(keys(test_instance.targets))
    for v in collect(keys(test_instance.depots))
        push!(all_vertices,v)
    end
    test_tour=construct_tour_LKH(test_instance,all_vertices)
    test_tour_cost=path_cost(test_instance,test_tour)    
    println("TEST TOUR COST: ",test_tour_cost)

    for alpha in [0.5/test_tour_cost, 1/test_tour_cost, 2/test_tour_cost]
        # Fixing the tau value
        for t in [0.5, 1, 2]
            # alpha=1/test_tour_cost
            println("ALPHA: ",alpha)

            instance=Instance(data,alpha,t)
            println("Instance alpha value is ", instance.alpha)
            #println("TEST: ",instance.cost_traversal[][1])
            # solver=Solver(instance)
            targets_covered = [i for i in keys(data.targets)]
            # Running the single vehicle TSP function, and computing the objective function
            # value
            tour, dwell_times, tour_cost, keys_arr, TSP_comp_time, dwell_time_comp_time = single_vehicle_TSP_dwell_times(instance, targets_covered, 1, true, true)
            println("DONE SOLVING")
            #println(tour, dwell_times, tour_cost, "Computation time is",  TSP_comp_time, dwell_time_comp_time)
            # println(dwell_times)
            # println("\tSTART TIME:",solver.start_time)
            # println("\tPROGRESS:",sort(solver.progress))
            # println("\tINITIAL OBJ:\t",solver.initial_obj)
            # println("\tLOCAL SRCH OBJ:\t",solver.local_search_obj)
            # println("\tFINAL OBJ:\t",solver.perturbation_obj)

            # Constructing route object corresponding to final solution
            route = Route(instance)
            println("Updating the dwell times.")
            # Updating the dwell times
            for i in keys_arr
                # println(i)
                route.vhcl_dwell_times[1][i] = dwell_times[i]
            end
            println("Computing the objective value")
            # Computing the objective function value for the vehicle
            neg_log = dwell_time_info_gain_func(dwell_times, [instance.tau for i in targets_covered], instance.alpha)
            # this is G*e^(-alpha*(dwell times sum))*e^(-alpha*(TSP cost))
            objective_value = exp(-neg_log)*exp(-instance.alpha*tour_cost)

            # sets this
            route = route_set_vhcl_i_tour_tour_cost(route, tour, tour_cost, objective_value, 1)

            # route=solver.final_solution
            if !check_if_valid(route)
                println("ERROR: INVALID SOLUTION")
                exit()
            end
            dwell_times=Float64[]
            for vhcl_no in 1:route.instance.dim_depots
                tour=route.vhcl_tours[vhcl_no]
                if length(tour)>=3
                    targets=tour[2:length(tour)-1]
                    for t in targets
                        push!(dwell_times,route.vhcl_dwell_times[vhcl_no][t])
                    end
                end
            end
            minn=min(dwell_times...)
            menan=sum(dwell_times)/length(dwell_times)
            maxx=max(dwell_times...)
            if length(dwell_times)%2==1
                mediann=dwell_times[div(length(dwell_times),2)]
            else
                mediann=dwell_times[div(length(dwell_times),2)]+dwell_times[div(length(dwell_times),2)-1]
                mediann=mediann/2
            end
            #println(dwell_times)
            # final_time=collect(keys(sort(solver.progress)))
            # final_time=final_time[length(final_time)]

            #string formatting
            #println("\tFINAL SOLN:",solver.final_solution)
            # now just put them in files 
            if !warmup
                write_table_entry(file,instance.dim_targets,
                            tour_cost,alpha,t,minn,mediann,menan,maxx,TSP_comp_time,dwell_time_comp_time)
            end
        end
    end
    warmup=false
end

write_table_tail()
close(f)