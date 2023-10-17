using Formatting

# Defining the template for writing onto a .tsp file to run LKH over.
template = """NAME: name
TYPE: TSP
COMMENT: name
DIMENSION: n_cities
EDGE_WEIGHT_TYPE: EXPLICIT
EDGE_WEIGHT_FORMAT: LOWER_DIAG_ROW
EDGE_WEIGHT_SECTION
matrix_sEOF"""

function construct_tour_LKH(instance, vertices_covered, precision = 2.0)
    # In this function, LKH is run for the considered instance. This is the main
    # function for running LKH.
    
    if length(vertices_covered)<4
        v=deepcopy(vertices_covered)
        push!(v,v[1])
        return v
    end

    # Constructing the weight matrix
    matrix = weight_matrix_construction(instance, vertices_covered, precision)
    # Constructing the .TSP file
    filename = raw"tmp\myRoute.tsp"
    # Opening the .tsp file
    f = open(filename, "w")
    # Writing onto the .tsp file
    write(f, create_tsplib_string(matrix))
    close(f)
    # Running LKH
    tour = run_lkh(filename, vertices_covered, precision)

    # Saving the obtained route
    route = tour["tour"]
    return route

end

function weight_matrix_construction(instance, vertices_covered, precision = 2.0)
    # In this function, the weight matrix corresponding to the instance is constructed.
    # NOTE: If an error is obtained while running LKH, reduce the precision parameter.
    # Here, vertices covered includes the depot of the vehicle as the first vertex.

    matrix = []
    # max_val = 0

    for i in vertices_covered
        row = []
        for j in vertices_covered
            if i == j
                append!(row, 0)
            else
                # Obtaining the cost of travelling from i to j, and integerizing it
                cost = floor(Int, instance.cost_traversal[min(i, j)][max(i, j)]*10^precision)
                append!(row, cost)
                # max_val = max(max_val, cost)
            end
        end
        push!(matrix, row)
    end

    return matrix
end

function create_tsplib_string(matrix, name = "route")
    # In this function, the .tsp file corresponding to "matrix" is constructed.
    # Note that "matrix" is a lower triangular matrix and specifies the cost to
    # go from one vertex to another.
    n_cities = size(matrix)[1]
    # # Obtaining "length" corresponding to the maximum value in the matrix. For example,
    # # is maximum value in the matrix is 12000, the length would be 5
    # width = length(string(max_val)) + 1
    matrix_s = ""
    for i = 1: n_cities
        # println(matrix[i])
        for j in 1:i
            matrix_s *= " " * string(matrix[i][j])
            # matrix_s += " ".join(["{0:>{1}}".format((int(elem)), width)
            #                     for elem in row[:i+1]])
        end
        matrix_s *= "\n"
    end

    # Modifying the template
    tmp = template
    tmp = replace(tmp, "name" => name)
    tmp = replace(tmp, "n_cities" => n_cities)
    tmp = replace(tmp, "matrix_s" => matrix_s)

    return tmp
end

function create_lkh_tsp_file_par_out_file(tsp_file_and_path, runs = 4)
    # In this function, the .par file and .out file for LKH will be created.
    # This is required to run LKH for the considered graph.
    
    # Removing the ".tsp" from the name of the file
    prefix, _ = split(tsp_file_and_path, ".")
    # Obtaining the name of the .par and .out files
    par_path = prefix * ".par"
    out_path = prefix * ".out"
    # Constructing the strings to be written onto the .par file
    par = """PROBLEM_FILE = str1
    RUNS = str2
    TOUR_FILE =  str3
    """
    par = replace(par, "str1" => tsp_file_and_path)
    par = replace(par, "str2" => string(runs))
    par = replace(par, "str3" => out_path)
    # Opening the .par file and writing the constructed string
    f = open(par_path, "w")
    write(f, par)
    close(f)

    return par_path, out_path

end

function run_lkh(tsp_file_and_path, vertices_covered, precision = 1.0)
    #= In this function, LKH is run over the constructed .tsp file. Here,
    tsp_file_and_path contains the relative directory in which the .tsp file is
    constructed, and the name of the .tsp file. vertices_covered denotes the list
    of vertices covered by the vehicle, including the depot =#
    
    # Obtaining the paths for the .par and .out file; also creating the .par file
    par_path, out_path = create_lkh_tsp_file_par_out_file(tsp_file_and_path)

    # Running the LKH file
    read(Cmd(["LKH-2", par_path]));

    # Opening the output file, and obtaining the tour
    meta = []
    raw = []
    f = open(out_path, "r")
    # Creating a flag variable to keep track of when the tour starts
    header = true
    lines = readlines(f) # Obtaining all lines in the output file
    solution = nothing

    for line in lines # Running through line by line
        # Checking when the tour starts
        if header
            if startswith(line, "COMMENT : Length = ")
                # Obtaining the cost of the tour, and scaling down using the
                # precision parameter. Note that the precision parameter was
                # used for integerizing the edge weights.
                solution = parse(BigInt, (last(split(line, " "))))/(10^precision)
            end
            if startswith(line, "TOUR_SECTION")
                header = false
                continue
            end
            append!(meta, line)
            # println(line)
            continue
        else
            if startswith(line, "-1") # Checking when the tour stops
                append!(raw, vertices_covered[1])
                break
            else
                append!(raw, vertices_covered[parse(Int64, strip(line, [' ']))])
                 # Appending the vertex, after removing
                # unnecessary spaces. Note that the obtained vertex is mapped
                # with "vertices_covered" here. 
                # The tour obtained from LKH corresponds to vertices being covered being 1, 2, ....
                # However, if our list of vertices to be covered is different from 1, 2, ..., we
                # need to match the corresponding vertices. For example, "1" in the tour will be
                # matched with vertices_covered[1], "2" will be matched with vertices_covered[2],
                # and so on.
            end
        end
    end

    # Collecting all initial information about the graph for which tour was constructed
    metadata = join(meta, "")

    # Closing the file
    close(f)

    # # Converting the obtained tour by matching to "vertices_covered". Note that
    # # the tour obtained from LKH corresponds to vertices being covered being 1, 2, ....
    # # However, if our list of vertices to be covered is different from 1, 2, ..., we
    # # need to match the corresponding vertices. For example, "1" in the tour will be
    # # matched with vertices_covered[1], "2" will be matched with vertices_covered[2],
    # # and so on.
    # raw_modified = []
    # for i in raw
    #     append!(raw_modified, vertices_covered[i])
    # end
    # # println(raw_modified)

    return Dict("tour" => raw, "solution" => solution, "metadata" => metadata)

end