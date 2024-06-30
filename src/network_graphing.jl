include("route.jl")

PYTHON_COMMAND="python"
PYTHON_FILE="src//network_graphing.py"


function graph_route(route,output_file,temp_file_name="temp//TEMP.txt",cleanup=false)
    # graphs the route object given
    # output file is where to save it
    # temp file is the temporary file made to interface with python graph_route.py
    # cleanup is whether to delete temp_file after
    
    temp_file=pwd()*"\\"*temp_file_name
    # since this is being read by python, we can just format this whole file as a python dict
    f=open(temp_file,"w")
    write(f,"{\n")

        write(f,"'depots':{")
        for k in keys(route.instance.depots)
            write(f,string(k))
            write(f,":")
            write(f,string(route.instance.depots[k]))
            write(f,",")
        end
        write(f,"},\n")

        write(f,"'targets':{")
        for k in keys(route.instance.targets)
            write(f,string(k))
            write(f,":")
            write(f,string(route.instance.targets[k]))
            write(f,",")
        end
        write(f,"},\n")

        write(f,"'tours':{")
        for vhcl_no in 1:route.instance.dim_depots
            tour=route.vhcl_tours[vhcl_no]
            write(f,string(vhcl_no)*":[")
            for v in tour
                write(f,string(v))
                write(f,",")
            end
            write(f,"],")
        end
        write(f,"},\n")
        write(f,"'dwell_times':{")
        for vhcl_no in 1:route.instance.dim_depots
            vertices=route.vhcl_tours[vhcl_no]
            if length(vertices)>1
                vertices=vertices[2:length(vertices)-1]
            end
            write(f,string(vhcl_no)*":")
            if length(vertices)==0
                write(f,"0")
            else
                avg_dwell=sum([route.vhcl_dwell_times[vhcl_no][v] for v in vertices])/length(vertices)
                write(f,string(avg_dwell))
            end
            write(f,",")
        end
        write(f,"},\n")
        write(f,"'vhcl_no_to_depot':{")
        for vhcl_no in 1:route.instance.dim_depots
            write(f,string(vhcl_no))
            write(f,":")
            depot=collect(keys(route.instance.depots))[vhcl_no]
            write(f,string(depot))
            write(f,",")
        end
        write(f,"},\n")

    write(f,"}")
    close(f)
    run(`$PYTHON_COMMAND $PYTHON_FILE $temp_file $output_file $cleanup`)
end
