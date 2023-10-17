using Classes, OrderedCollections

# In this file, a class object for storing the data is created.
@class mutable Data begin
    name_targets::String
    name_depots::String
    path::String
    targets::OrderedDict
    depots::OrderedDict
    dim_targets::Int64
    dim_depots::Int64

    # Writing a construction function
    function Data(name, path)

        # ----------Reading the .TSP file containing information about the targets
        # Changing directory to retrieve the .TSP file
        # Obtaining the current directory
        curr_dir = pwd()
        # Changing directory to the folder containing the data
        new_dir = curr_dir * path
        cd(new_dir)
        # Reading the file containing information about the targets and depots
        lines_targets = readlines(name * "_targets.tsp")
        lines_depots = readlines(name * "_depots.tsp")
        # Returning to the original directory
        cd(curr_dir)

        # Declaring variables for coordinates of targets, depots, and number of targets
        # and depots
        dim_targets = 0; dim_depots = 0; targets = Dict(); depots = Dict();

        # ------------Extracting information about the dataset from the files
        for i = 1:2 # Reading from file and obtaining information about the targets and depots
            if i == 1 # Reading from file corresponding to the targets
                lines = lines_targets
            else # Reading from file corresponding to the depots
                lines = lines_depots
            end

            dim = 0 # Variable for storing number of targets/depots
            flag = 0 # Flag variable for determining if information about targets/depots
            # about to start in the file
            info = Dict() # Variable for storing coordinates

            # Running through each line in the file
            for line in lines
                # Obtaining the number of targets or depots
                if occursin("DIMENSION", line)
                    tmp = last(split(strip(line, [' ']), ":", keepempty = false))
                    dim = parse(Int64, strip(tmp, [' ']))
                
                # Obtaining coordinates of targets or depots
                elseif occursin("NODE_COORD_SECTION", line) # Data about the nodes starts from
                    # the next line
                    flag = 1
                    continue

                elseif occursin("EOF", line) # Breaking the loop if end of file is reached

                    break

                end

                # Reading the coordinates of each node
                if flag == 1

                    tmp = split(strip(line, [' ']), " ", keepempty = false)
                    info[parse(Int64, tmp[1])] = (parse(Float64, tmp[2]), parse(Float64, tmp[3]))
                    
                end

            end

            if i == 1
                dim_targets = dim
                targets = info
            else
                dim_depots = dim
                depots = info
                #println(info)
            end
        end
        targets=sort(targets)
        depots=sort(depots)
        # println(dim_targets, dim_depots)
        return new(name * "_targets.tsp", name * "_depots.tsp", path, targets,
         depots, dim_targets, dim_depots)
    end

end