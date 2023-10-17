# -*- coding: utf-8 -*-
"""
Created on Wed Feb 22 09:35:02 2023

@author: deepa
"""

import numpy as np
import networkx as nx
import os
from itertools import combinations
import math
import copy
import gurobipy as gp
from gurobipy import GRB

# Retrieving function to run the LKH heuristic for each vehicle
os.chdir('D:\TAMU\Research\Min-max TSP\Codes\Multi-vehicle-TSP')
from min_sum_TSP_functions import LKH_each_vehicle

# Returning to the original directory
os.chdir('D:\TAMU\Research\Min-max TSP\Codes\Min-max depot symmetric TSP branch and cut')

def MD_algorithm_soln_for_IP(G_comp_arr, T, dep, term):
    '''
    In this function, the solution obtained from running the MD algorithm is
    returned to provide an initial feasible solution for the MILP.

    Parameters
    ----------
    G_comp_arr : TYPE
        DESCRIPTION.
    T : TYPE
        DESCRIPTION.
    dep : TYPE
        DESCRIPTION.
    term : TYPE
        DESCRIPTION.

    Returns
    -------
    TYPE
        DESCRIPTION.

    '''
    
    # Creating arrays to store the phi and edge values for the feasible solution
    phi_vals_feasible = []
    x_vals_feasible = []
    
    # Obtaining the solution from the MD algorithm
    obj_tours, targ_allocation_arr, vertices_tours, obj_tours_ini, _, _, _, _ = MD_algorithm(G_comp_arr, T, dep)
    
    # Running through each vehicle and obtaining the phi values
    for i in range(len(G_comp_arr)):
        
        # Running through the targets for each vehicle and obtaining the phi values
        # which keep track of the targets allocated to each vehicle
        phi_vals_vhcl = {} # Creating a dictionary to store the phi values    
    
        for t in T:
            
            # Checking if the considered target is in the tour of the vehicle or not
            if t in targ_allocation_arr[i]:
                
                phi_vals_vhcl[t] = 1
                
            else:
                
                phi_vals_vhcl[t] = 0
                
        # Appending the phi-values for the vehicle to the array
        phi_vals_feasible.append(phi_vals_vhcl)
        
        # Obtaining the x-values for each vehicle, i.e., the edges traversed
        # in each vehicle's tour obtained from the MD algorithm
        x_vals_vhcl = {} # Creating a dictionary to store the x values
        
        # First, obtaining the list of edges in the vehicle's tour, except for the last edge
        # Checking if atleast one target is allocated to the considered vehicle
        if len(targ_allocation_arr[i]) == 0:
            
            edges_tour = []
        else:
            
            edges_tour = [(vertices_tours[i][j - 1], vertices_tours[i][j]) for j in range(1, len(vertices_tours[i]) - 1)]
            # The last edge connects the last target covered by the vehicle to the depot.
            # Instead, we connect it to the terminal.
            edges_tour.append((vertices_tours[i][-2], term[i]))
            # Adding an edge between the terminal and the depot
            edges_tour.append((dep[i], term[i]))
        
        # Now, we obtain the x-values for the vehicle. First, we obtain the x-values
        # for edges between targets
        for (k, l) in combinations(T, 2):
            
            if (k, l) in edges_tour or (l, k) in edges_tour:
                
                x_vals_vhcl[min(k, l), max(k, l)] = 1
                
            else:
                
                x_vals_vhcl[min(k, l), max(k, l)] = 0
                
        # Next, we obtain the x-values for the edges between the targets and
        # the depot/terminal
        for t in T:
            
            for d in [dep[i], term[i]]:
                
                if (t, d) in edges_tour or (d, t) in edges_tour:
                    
                    x_vals_vhcl[min(t, d), max(t, d)] = 1
                    
                else:
                    
                    x_vals_vhcl[min(t, d), max(t, d)] = 0
                    
        # Adding an edge between depot and terminal
        if (dep[i], term[i]) in edges_tour or (term[i], dep[i]) in edges_tour:

            x_vals_vhcl[min(dep[i], term[i]), max(dep[i], term[i])] = 1

        else:

            x_vals_vhcl[min(dep[i], term[i]), max(dep[i], term[i])] = 0
        
        # Appending the x-values for the vehicle to the array
        x_vals_feasible.append(x_vals_vhcl)
        
    return max(obj_tours), x_vals_feasible, phi_vals_feasible, max(obj_tours_ini), min(obj_tours_ini)

def MD_algorithm(G_comp_arr, T, dep, mode = 1, angle_perturbations = False):
    '''
    In this function, the MD algorithm is implemented for the graph provided.

    Parameters
    ----------
    G_comp_arr : TYPE
        DESCRIPTION.
    T : TYPE
        DESCRIPTION.
    dep : TYPE
        DESCRIPTION.

    Returns
    -------
    obj_tours : TYPE
        DESCRIPTION.
    targ_allocation_arr : TYPE
        DESCRIPTION.
    vertices_tours : TYPE
        DESCRIPTION.
    obj_tours_ini : TYPE
        DESCRIPTION.

    '''
    
    # Obtaining the modified graph after potential perturbation for depots that
    # contain more than one vehicle
    G_comp_arr_mod = initialization_MD_algorithm_graph_depot_perturbation(G_comp_arr, T, dep)
    
    # Obtaining initial target allocation by solving the assignment problem, so
    # that all vehicles have nearly the same number of targets to be covered
    targ_allocation_arr_ini =\
        initialization_MD_algorithm_solving_assignment_problem(G_comp_arr_mod, T, dep)
    # print('Initial target allocation is ' + str(targ_allocation_arr_ini))
        
    # Obtaining the initial feasible solution by running LKH for each vehicle using
    # the obtained initial target allocation
    obj_tours_ini, vertices_tours_ini =\
        MD_algorithm_constructing_tours_LKH(G_comp_arr, targ_allocation_arr_ini, dep,\
                                            [i for i in range(len(G_comp_arr))])
            
    # print('Initial tours obtained for the vehicles are of cost ' + str(obj_tours_ini) +\
    #        ', and the tours are ' + str(vertices_tours_ini) + '.')
        
    # Obtaining the improved solution after local search
    if mode == 1:
    
        obj_tours, targ_allocation_arr, vertices_tours =\
            local_search_maximal_route(G_comp_arr, targ_allocation_arr_ini, dep,\
                                       obj_tours_ini, vertices_tours_ini)
                
    elif mode == 2:
        
        obj_tours, targ_allocation_arr, vertices_tours =\
            local_search_maximal_route_switch_and_swap(G_comp_arr, targ_allocation_arr_ini, dep,\
                                       obj_tours_ini, vertices_tours_ini)
            
    # print('Tours obtained after local search are of cost ' + str(obj_tours) +\
    #       ', and the tours are ' + str(vertices_tours) + '.')
            
    # Obtaining the solution after perturbation
    obj_tours_perturb, targ_allocation_arr_perturb, vertices_tours_perturb, angle_perturbations =\
        perturbation_solution(G_comp_arr, T, targ_allocation_arr, dep, obj_tours,\
                              vertices_tours, mode, angle_perturbations)
            
    # print('Tours obtained after perturbation are of cost ' + str(obj_tours_perturb) +\
    #       ', and the tours are ' + str(vertices_tours_perturb) + '.')
            
    return obj_tours_perturb, targ_allocation_arr_perturb, vertices_tours_perturb, obj_tours_ini,\
        obj_tours, targ_allocation_arr, vertices_tours, angle_perturbations

def initialization_MD_algorithm_graph_depot_perturbation(G_comp_arr, T, dep):
    '''
    In this function, the depots are perturbed in case there are multiple vehicles
    at a single depot.

    Parameters
    ----------
    G_comp_arr : TYPE
        DESCRIPTION.
    T : TYPE
        DESCRIPTION.
    dep : TYPE
        DESCRIPTION.

    Returns
    -------
    G_comp_arr_mod : TYPE
        DESCRIPTION.

    '''
    
    # Retrieving the position of the depots
    pos_depots = {}
    for i in range(len(dep)):
        
        pos_depots[dep[i]] = G_comp_arr[i].nodes(data = True)[dep[i]]['pos']
        
    # Checking if there are multiple vehicles present at a depot
    G_temp = nx.Graph()
    # Running through each pair of depots, and checking if they are within a distance
    # of tolerance 0.1. If so, the pair is added to the G_temp with an edge. In this
    # manner, we can detect each depot that contains more than one vehicle.
    
    for (i, j) in list(combinations(list(pos_depots.keys()), 2)):
        
        # Obtaining the distance between depots i and j
        dist = math.sqrt((pos_depots[i][0] - pos_depots[j][0])**2\
                         + (pos_depots[i][1] - pos_depots[j][1])**2)
        
        if dist <= 10**(-3): # tolerance used to check if two vehicles start from nearly 
        # the same depot
        
            G_temp.add_edge(dep.index(i), dep.index(j))
            
    # Checking if there depots that are sufficiently close to each other using the
    # graph G_temp constructed
    # Updating pos_depots by considering each set of depots that are sufficiently close
    # to each other
    
    if nx.number_connected_components(G_temp) > 0:
        
        # print('Connected components in G_temp are ' + str(list(nx.connected_components(G_temp))))
        
        # Running through the depots in each connected component in G_temp
        for conn_comp in list(nx.connected_components(G_temp)):
            
            # print('Connected component considered is ' + str(conn_comp))
            
            # Obtaining the position of the centroid all depots in the connected_component
            x_centroid_loc = sum([pos_depots[dep[i]][0] for i in conn_comp])/len(conn_comp)
            y_centroid_loc = sum([pos_depots[dep[i]][1] for i in conn_comp])/len(conn_comp)
            print(x_centroid_loc)
            print(y_centroid_loc)
            
            # Generating the array of angles to place the new perturbed depots
            # Generating an initial random angle for the perturbation
            rand_angle = np.random.rand()*2*math.pi
            angle = np.linspace(rand_angle, 2*math.pi + rand_angle, len(conn_comp), endpoint = False)
            
            # Modifying the position of the initial depots with the perturbed depots
            for i in range(len(conn_comp)):
                
                print(list(conn_comp)[i])
                pos_depots[dep[list(conn_comp)[i]]] = (x_centroid_loc + 0.1*math.cos(angle[i]),\
                                                       y_centroid_loc + 0.1*math.sin(angle[i]))
                    
    # Generating the modified array of graphs to run LKH over after (possibly) perturbing
    # the depots
    G_comp_arr_mod = []
    
    # Constructing the graph for each vehicle
    for i in range(len(dep)):
        
        G_temp = nx.Graph()
        
        # Adding edges that go between targets
        for (j, k) in combinations(T, 2):
            
            G_temp.add_edge(min(j, k), max(j, k), weight = G_comp_arr[i][j][k]['weight'])
            
        # Adding edges between the targets and the depot
        for j in T:
            
            # Computing the distance between the target and the depot (after potential
            # perturbation)
            dist = math.sqrt((G_comp_arr[i].nodes(data = True)[j]['pos'][0] - pos_depots[dep[i]][0])**2\
                             + (G_comp_arr[i].nodes(data = True)[j]['pos'][1] - pos_depots[dep[i]][1])**2)
                
            G_temp.add_edge(min(j, dep[i]), max(j, dep[i]), weight = dist)
            
        # Appending the graph to G_comp_arr_mod
        G_comp_arr_mod.append(G_temp)
            
    return G_comp_arr_mod

def initialization_MD_algorithm_solving_assignment_problem(G_comp_arr_mod, T, dep):
    '''
    In this function, the initial assignment problem is solved, which is the LP-based
    assignment, to provide an initial feasible solution for the MD algorithm.

    Parameters
    ----------
    G_comp_arr_mod : Array
        G_COMP_ARR_MOD IS THE MODIFIED ARRAY CONTAINING THE GRAPHS FOR EACH VEHICLE
        AFTER POTENTIAL PERTURBATION OF DEPOTS THAT CONTAIN MORE THAN ONE VEHICLE.
    T : TYPE
        DESCRIPTION.
    dep : TYPE
        DESCRIPTION.

    Raises
    ------
    Exception
        DESCRIPTION.

    Returns
    -------
    targ_allocation_arr : TYPE
        DESCRIPTION.

    '''
        
    # Setting up the Gurobi model to solve the LP
    assign_model = gp.Model()
    
    # Declaring variables corresponding to the edges connecting a target with the
    # depot for a vehicle
    num_vhcls = len(G_comp_arr_mod) # Obtaining the number of vehicles
    
    # Declaring an array to store the edge variables for the graph corresponding
    # to each vehicle
    model_edge_vars_arr = []
    
    for k in range(num_vhcls):
        
        model_kth_vhcl_vars_dict = {(min(i, j), max(i, j)) for i in T for j in [dep[k]]}
        # obj_val_vars = [G_comp_arr_mod[k][i][j]['weight'] for i in T for j in [dep[k]]]
        
        # Variables for the edges
        var_name = 'x' + str(k + 1)
        model_kth_vhcl_edge_vars =\
            assign_model.addVars(model_kth_vhcl_vars_dict, vtype = GRB.CONTINUOUS,\
                                 name = var_name, lb = 0, ub = 1)
        model_edge_vars_arr.append(model_kth_vhcl_edge_vars)
        
        # Setting the values of the variables in the objective function        
        for e in model_kth_vhcl_vars_dict:
            
            model_edge_vars_arr[k][e[0], e[1]].Obj = G_comp_arr_mod[k][e[0]][e[1]]['weight']
        
    # Making a copy of the variables corresponding to the edges in the other direction    
    for k in range(num_vhcls):
        
        for i, j in model_edge_vars_arr[k].keys():
            
            # edge in opposite direction
            model_edge_vars_arr[k][j, i] = model_edge_vars_arr[k][i, j]
            
    # Target allocation constraint - each target is allocated to a depot
    for i in T:
            
        cons_edges = sum([model_edge_vars_arr[j][i, dep[j]] for j in range(num_vhcls)])
            
        assign_model.addConstr(cons_edges == 1)
        
    # Determining number of targets assigned to each vehicle
    q, rem = divmod(len(T), num_vhcls)
    # The first "rem" number of vehicles are assigned with q + 1 targets, and the
    # remaining vehicles are assigned q targets. In this manner, we get rem + q*num_vhcls
    # number of targets in total, which is equal to the total number of targets considered.
    for k in range(num_vhcls):
        
        # Obtaining sum of edges corresponding to the (k + 1)th vehicle
        cons_edges = sum([model_edge_vars_arr[k][i, dep[k]] for i in T])
        
        # Checking if (k + 1) <= rem, i.e., if the vehicle considered is among the
        # first "rem" number of vehicles. In this case, the vehicle is allocated
        # q + 1 number of targets; else, q targets.
        if (k + 1) <= rem:
            
            assign_model.addConstr(cons_edges == q + 1)
            
        else:
            
            assign_model.addConstr(cons_edges == q)
    
    assign_model.write('test.lp')
    
    # Solving the model
    assign_model.optimize()
        
    # Retrieving the allocation of targets
    targ_allocation_arr = []
    # Running through the x variables for each vehicle in the solution
    for i in range(num_vhcls):
    
        x_vals = {k: v.X for k, v in model_edge_vars_arr[i].items()}
        
        # Running through each entry in x_vals to determine which targets have 
        # been allocated to the considered vehicle
        targ_allocation_vhcl = []
        for (t, d), val in x_vals.items():
            
            # Checking if d = depot of the vehicle, since in x_vals, we will
            # have edges of form (l, m) and (m, l), i.e., repeated edges. We
            # want to check if target is visited in one type of edge, wherein
            # the second node in the edge is the depot
            
            if d == dep[i]:
                
                # Checking if the value is nearly one
                if val >= 1 - 10**(-3):
                    
                    # In this case, the target t is allocated to the considered vehicle
                    targ_allocation_vhcl.append(t)
                    
                # Checking if the value is fractional
                elif val >= 10**(-3):
                    
                    print(x_vals)
                    raise Exception('Fractional value obtained in the solution.')
                    
        # Appending the target allocation array for the considered vehicle
        targ_allocation_arr.append(targ_allocation_vhcl)
        
    return targ_allocation_arr

def MD_algorithm_constructing_tours_LKH(G_comp_arr, targ_allocation_arr, dep,\
                                        vhcls_index_tour_construction, precision = 4,\
                                        rel_direct = "LKH_data_files"):
    '''
    In this function, the tours are constructed after obtaining a target assignement
    using LKH.

    Parameters
    ----------
    G_comp_arr : Array
        G_COMP_ARR IS THE ORIGINAL ARRAY CONTAINING THE GRAPHS FOR EACH VEHICLE.
    targ_allocation_arr : TYPE
        DESCRIPTION.
    dep : TYPE
        DESCRIPTION.
    vhcls_index_tour_construction : Array
        Vehicle indices for which the tours must be reconstructed.
    precision : TYPE, optional
        DESCRIPTION. The default is 4.
    rel_direct : TYPE, optional
        DESCRIPTION. The default is "LKH_data_files".

    Returns
    -------
    obj_tours : TYPE
        DESCRIPTION.
    vertices_tours : TYPE
        DESCRIPTION.

    '''
        
    # print('Tour construction.')
    # Keeping track of tour costs for each vehicle
    obj_tours = []
    # # Keeping track of the tour for each vehicle, wherein the sequence of edges
    # # taken is kept track of
    # edges_tours = []
    # Keeping track of the tour for each vehicle, wherein the sequence of vertices
    # taken is kept track of
    vertices_tours = []
        
    # Constructing the tours for only the provided list of vehicles in vhcls_index_tour_construction
    for i in vhcls_index_tour_construction:
        
        tour_vertices_vehicle = []
        
        # Checking if no targets are allocated to chosen vehicle
        if len(targ_allocation_arr[i]) == 0:
            
            tour_vertices_vehicle = []
            
            # Updating obj_tours with the tour cost for the considered vehicle
            obj_tours.append(0)
        
        # Checking if there is only one target allocated to the considered vehicle
        elif len(targ_allocation_arr[i]) == 1:
            
            # tour_edges_vehicle = [(dep[i], targ_allocation_arr[i][0]),\
            #                       (targ_allocation_arr[i][0], dep[i])]
            tour_vertices_vehicle = [dep[i], targ_allocation_arr[i][0], dep[i]]
                
            # Updating obj_tours with the tour cost for the considered vehicle
            obj_tours.append(round(2*G_comp_arr[i][targ_allocation_arr[i][0]][dep[i]]['weight'], precision))
            
        # Checking if there are only two targets allocated to the considered vehicle
        elif len(targ_allocation_arr[i]) == 2:
            
            # Since G_comp_arr[i] is an undirected graph, we can construct the
            # tour directly.
            # tour_edges_vehicle = [(dep[i], targ_allocation_arr[i][0]),\
            #                       (targ_allocation_arr[i][0], targ_allocation_arr[i][1]),\
            #                       (targ_allocation_arr[i][1], dep[i])]
            tour_vertices_vehicle = [dep[i], targ_allocation_arr[i][0],\
                                     targ_allocation_arr[i][1], dep[i]]
            
            # Updating obj_tours with the tour cost for the considered vehicle
            obj_tours.append(round(G_comp_arr[i][targ_allocation_arr[i][0]][dep[i]]['weight']\
                             + G_comp_arr[i][targ_allocation_arr[i][1]][dep[i]]['weight']\
                             + G_comp_arr[i][targ_allocation_arr[i][0]][targ_allocation_arr[i][1]]['weight'], precision))
                
        # If more than two targets are allocated to the considered vehicle, LKH is
        # run
        else:
            
            # Constructing the weight matrix to be used for LKH
            # Appending the list of targets allocated to the vehicle
            vertices_visited = copy.deepcopy(targ_allocation_arr[i])
            # Adding the depot as a vertex to be covered. The insertion of the
            # depot is done to the beginning of this array
            vertices_visited.insert(0, dep[i])
            # print(vertices_visited)
            
            weight_matrix = np.empty((len(vertices_visited), len(vertices_visited)))
            
            # Obtaining the cost of travelling between two targets, multiplying by the
            # precision factor chosen, and rounding to an integer, as LKH requires the
            # edge weights to be an integer
            # for k in range(len(vertices_visited)):
            #     for l in range(len(vertices_visited)):
                    
            #         if vertices_visited[k] == vertices_visited[l]:
                        
            #             # Providing a very high cost to travel between the same vertices
            #             weight_matrix[k, l] = 10**4*(10**precision)
                        
            #         else:
                        
            #             weight_matrix[k, l] =\
            #                 int(G_comp_arr[i][vertices_visited[k]][vertices_visited[l]]["weight"]*(10**precision))
                
            # Providing a very high cost to travel between the same vertices
            for k in range(len(vertices_visited)):
                
                weight_matrix[k, k] = 10**4*(10**precision)
                
            for (k, l) in combinations(range(len(vertices_visited)), 2):
                
                cost = round(G_comp_arr[i][vertices_visited[k]][vertices_visited[l]]["weight"]*(10**precision))
                weight_matrix[k, l] = cost
                weight_matrix[l, k] = cost                            
                            
            # Running LKH over the created weight matrix
            # Choosing the name of the files for the .par file (to run LKH) and the output file            
            filename_par = "vehicle_" + str(i + 1)
            filename_out = "output_vehicle_" + str(i + 1)
            LKH_each_vehicle(weight_matrix, filename_par, filename_out, rel_direct)
            
            # Reading the output file obtained from running LKH
            # Changing the directory to the directory containing the output file
            orig_dir = os.getcwd()
            path = os.path.join(orig_dir, rel_direct)            
            os.chdir(path)
            
            # Reading the output file
            f = open(filename_out + ".txt", "r")
            part_of_tour_flag = False
            
            # Obtaining the tour cost and the tour of the vehicle            
            for curr_line in f:
                
                # Obtaining the cost of the tour
                if "COMMENT : Length = " in curr_line:
                
                    curr_line = curr_line.replace("COMMENT : Length = ", "")
                    tour_cost = int(curr_line)/(10**(precision))
                    
                # Obtaining the tour from the solution
                # NOTE: Vertex j in the tour obtained from LKH corresponds to vertices_visited[j - 1].
                # This is because LKH interprets the weight matrix to be corresponding to vertices
                # 1 to n, where n is the number of targets covered by the corresponding vehicle.
                if "TOUR_SECTION" in curr_line:
                    
                    part_of_tour_flag = True # This flag indicates information about the tour
                    # will start from the next line in the output text file
                    continue
                
                if part_of_tour_flag:
                    
                    # Current line is a target in the tour
                    target = int(curr_line)
                    
                    if target == -1:
                        
                        # Tour has been completed. The vehicle completes the tour
                        # by revisiting the vertex at which it started.
                        tour_vertices_vehicle.append(vertices_visited[0])
                        break
                    
                    else:
                                         
                        # Mapping the "target" in the tour obtained from LKH to the
                        # target in vertices_visited and appending.
                        tour_vertices_vehicle.append(vertices_visited[target - 1])
                        
                        
            # Closing the file
            f.close()
                
            # Updating the array containing the objective values
            obj_tours.append(tour_cost)
            
            # Returning to the original directory
            os.chdir(orig_dir)
            
        # Appending the vehicle's tour to the array
        vertices_tours.append(tour_vertices_vehicle)
            
    return obj_tours, vertices_tours

def savings_removing_target_max_route(G_vhcl, vhcl_tour):
    '''
    In this function, the proxy for savings that could be obtained by removing
    a particular target from the maximal vehicle's tour is computed.

    Parameters
    ----------
    G_vhcl : NetworkX Undirected Graph
        Contains the graph corresponding to the chosen vehicle.
    vhcl_tour : Array
        Contains the tour of the chosen vehicle (which is the vehicle with maximum cost).

    Returns
    -------
    savings_sorted : Array
        Contains the sorted list of targets in decreasing order of proxy savings.
    '''
    
    # Running through the tour of the vehicle, except for the start and the end
    # of the array, as they are the depot
    # Creating a dictionary, and storing the savings obtained by removing the target
    # from the considered maximal route
    savings = {}
    
    for i in range(1, len(vhcl_tour) - 1):        
        
        # Obtaining the cost of the edge that would connect the vertices before
        # and after the considered target
        # If the two vertices are the same, which occurs when the vehicle has a
        # tour with just one target, then the cost of edge that replaces in the tour
        # is zero.
        if vhcl_tour[i - 1] == vhcl_tour[i + 1]:
        
            cost_fin_edge_replacement = 0
            
        else:
            
            cost_fin_edge_replacement = G_vhcl[vhcl_tour[i - 1]][vhcl_tour[i + 1]]['weight']
        
        # Computing the savings and storing in the dictionary
        savings[vhcl_tour[i]] = G_vhcl[vhcl_tour[i - 1]][vhcl_tour[i]]['weight']\
            + G_vhcl[vhcl_tour[i]][vhcl_tour[i + 1]]['weight']\
            - cost_fin_edge_replacement
            
    # Sorting the list in the descending order of savings
    savings_sorted = sorted(savings.items(), key = lambda x:x[1], reverse = True)
    
    # print('Sorted savings for targets in maximal vehicle is ' + str(savings_sorted))
    
    return savings_sorted

def insertion_cost_routes(G_comp_arr, vertices_tours, max_tour_vhcl_index,\
                          target_insert, dep):
    '''
    In this function, the insertion cost associated with each vehicle (except the 
    vehicle with the maximum tour cost) for inserting a particular chosen target
    is computed. Further, the vehicle with the least cost of insertion is returned.

    Parameters
    ----------
    G_comp_arr : Array
        Contains the graphs for all vehicles.
    vertices_tours : Array
        Contains the current tours for all vehicles.
    max_tour_vhcl_index : Scalar
        Index corresponding to the vehicle with the maximum tour cost.
    target_insert : Scalar
        Target to be inserted in vehicles except for the vehicle with the maximum
        tour cost.

    Returns
    -------
    minimum_insertion_vhcl_index : Scalar
        Contains the index of the vehicle that has the least cost of insertion.
    minimum_insertion_cost_vhcls : Scalar
        Contains the cost of insertion corresponding to the vehicle with the least
        cost of insertion.
    '''
    
    # Declaring variables to store the vehicle with the minimum insertion cost
    minimum_insertion_cost_vhcls = np.infty
    minimum_insertion_vhcl_index = np.NaN
    
    # print('Obtaining insertion cost for target ' + str(target_insert))
    # Running through the tour for each vehicle and obtaining the least insertion cost
    for i in range(len(G_comp_arr)):
        
        # If the index i equals the index of the vehicle with the maximal tour,
        # then skip
        if i == max_tour_vhcl_index:
            
            continue
        
        else:
            
            # Keeping track of the minimum insertion cost for the considered
            # vehicle
            min_insertion_cost_vhcl_i = np.infty
            # Checking if tour of chosen vehicle is empty
            if len(vertices_tours[i]) == 0:
                
                min_insertion_cost_vhcl_i = 2*G_comp_arr[i][dep[i]][target_insert]['weight']
                
                # print('The cost of insertion for vehicle number ' + str(i + 1) +\
                #       ' between vertices ' + str(dep[i]) + ' and ' + str(dep[i]) +\
                #       ' is ' + str(min_insertion_cost_vhcl_i))
            
            else:
                
                # Running through the tour of the chosen vehicle
                for j in range(1, len(vertices_tours[i])):
                    
                    # print('Checking inserting before target ', vertices_tours[i][j])
                    vertex_before = vertices_tours[i][j - 1]
                    vertex_after = vertices_tours[i][j]
                    # Here, we try to insert the considered target between vertices
                    # vertices_tours[i][j - 1] and vertices_tours[i][j] for the ith
                    # vehicle. We first obtain the cost of the edge connecting
                    # these considered vertices in the vehicle's original tour.
                    if vertex_before == vertex_after:
                    
                        cost_edge_removal = 0 # In this case, both vertices
                        # are the depot
                        
                    else:
                        
                        cost_edge_removal = G_comp_arr[i][vertex_before][vertex_after]['weight']
                    
                    # Computing the cost of insertion
                    insertion_cost = G_comp_arr[i][vertex_before][target_insert]['weight']\
                        + G_comp_arr[i][target_insert][vertex_after]['weight']\
                        - cost_edge_removal
                        
                    # print('The cost of insertion for vehicle number ' + str(i + 1) +\
                    #       ' between vertices ' + str(vertex_before) + ' and ' + str(vertex_after) +\
                    #       ' is ' + str(insertion_cost))
                        
                    # Keeping track of the minimum cost of insertion
                    if insertion_cost < min_insertion_cost_vhcl_i:
                        
                        min_insertion_cost_vhcl_i = insertion_cost
                        
            # Updating the minimum insertion cost across vehicles
            if min_insertion_cost_vhcl_i < minimum_insertion_cost_vhcls:
                
                minimum_insertion_vhcl_index = i
                minimum_insertion_cost_vhcls = min_insertion_cost_vhcl_i
                
    return minimum_insertion_vhcl_index, minimum_insertion_cost_vhcls

def local_search_maximal_route(G_comp_arr, targ_allocation_arr_ini, dep,\
                               obj_tours_ini, vertices_tours_ini):
    
    flag = 0 # Flag variable to check if the solution can be improved or not
    
    obj_tours = copy.deepcopy(obj_tours_ini) # Tour cost for each vehicle, which
    # will be updated during the local search
    targ_allocation_arr = copy.deepcopy(targ_allocation_arr_ini) # Allocation of
    # targets to vehicles, which will be updated during the local search
    vertices_tours = copy.deepcopy(vertices_tours_ini) # Tours of the vehicles,
    # which would be updated during the local search
    
    while flag != 1: # Running the local search till no more improvement can be
    # obtained
    
        flag = 1 # Changing the flag variable, which will then be set to zero
        # only if an improvement is obtained
        
        # Obtaining the vehicle index corresponding to the maximum cost tour
        max_tour_vhcl_index = obj_tours.index(max(obj_tours))
        
        # Obtaining the savings obtained by removing a target from the maximal tour.
        # Note that the obtained list has been sorted in the descending order of savings.
        savings_sorted = savings_removing_target_max_route(G_comp_arr[max_tour_vhcl_index],\
                                                           vertices_tours[max_tour_vhcl_index])
            
        # Running through each target in the array savings_sorted
        array_targets = [i[0] for i in savings_sorted]
        for t in array_targets:
            
            # Obtaining the vehicle index with the minimum insertion cost for the
            # considered target
            min_insertion_vhcl_index, _ = insertion_cost_routes(G_comp_arr, vertices_tours,\
                                                                max_tour_vhcl_index, t, dep)
                
            # Modifying the target allocation array
            targ_allocation_arr_mod = copy.deepcopy(targ_allocation_arr)
            # Removing the considered target t from the vehicle with the maximal tour
            targ_allocation_arr_mod[max_tour_vhcl_index].remove(t)
            # Adding the considered target t to the vehicle with the minimum insertion
            # cost
            targ_allocation_arr_mod[min_insertion_vhcl_index].append(t)
            
            # print('Testing improvement.')
            # Resolving the tours for the vehicle with the maximum tour cost and
            # the vehicle with the minimum insertion cost
            obj_tours_two_vhcls, vertices_tours_two_vhcls =\
                MD_algorithm_constructing_tours_LKH(G_comp_arr, targ_allocation_arr_mod, dep,\
                                                    [max_tour_vhcl_index, min_insertion_vhcl_index])
                    
            # Checking if the newly obtained tours help reduce the maximum tour cost
            obj_tours_mod = copy.deepcopy(obj_tours)
            obj_tours_mod[max_tour_vhcl_index] = obj_tours_two_vhcls[0]
            obj_tours_mod[min_insertion_vhcl_index] = obj_tours_two_vhcls[1]
            
            # print('Modified target allocation array is ' + str(targ_allocation_arr_mod))
            # print('Objective obtained from trying to remove max target from LKH is ' +\
            #       str(obj_tours_mod) + ', whereas initial objective is ' + str(obj_tours))
            # print('New tour potentially obtained is ' + str(vertices_tours_two_vhcls))
            
            if max(obj_tours_mod) < max(obj_tours):
                
                obj_tours = obj_tours_mod
                targ_allocation_arr = targ_allocation_arr_mod
                vertices_tours[max_tour_vhcl_index] = vertices_tours_two_vhcls[0]
                vertices_tours[min_insertion_vhcl_index] = vertices_tours_two_vhcls[1]
                # print('Local improvement in solution obtained. New objective values are '\
                #       + str(obj_tours) + ', new target allocation is ' + str(targ_allocation_arr)\
                #       + ' and new tours are ' + str(vertices_tours) + '.')
                
                flag = 0 # Modifying the flag variable, since a local improvement
                # has been obtained
                
                break # Breaking from the loop in searching for replacing a target from
                # the maximum tour
                
    return obj_tours, targ_allocation_arr, vertices_tours

def local_search_maximal_route_switch_and_swap(G_comp_arr, targ_allocation_arr_ini, dep,\
                                               obj_tours_ini, vertices_tours_ini):
    
    flag = 0 # Flag variable to check if the solution can be improved or not
    
    obj_tours = copy.deepcopy(obj_tours_ini) # Tour cost for each vehicle, which
    # will be updated during the local search
    targ_allocation_arr = copy.deepcopy(targ_allocation_arr_ini) # Allocation of
    # targets to vehicles, which will be updated during the local search
    vertices_tours = copy.deepcopy(vertices_tours_ini) # Tours of the vehicles,
    # which would be updated during the local search
    
    while flag != 1: # Running the local search till no more improvement can be
    # obtained
    
        flag = 1 # Changing the flag variable, which will then be set to zero
        # only if an improvement is obtained
        
        # Obtaining the vehicle index corresponding to the maximum cost tour
        max_tour_vhcl_index = obj_tours.index(max(obj_tours))
        
        # Obtaining the savings obtained by removing a target from the maximal tour.
        # Note that the obtained list has been sorted in the descending order of savings.
        savings_sorted = savings_removing_target_max_route(G_comp_arr[max_tour_vhcl_index],\
                                                           vertices_tours[max_tour_vhcl_index])
            
        # Running through each target in the array savings_sorted
        array_targets = [i[0] for i in savings_sorted]
        for t in array_targets:
            
            # Obtaining the vehicle index with the minimum insertion cost for the
            # considered target
            min_insertion_vhcl_index, _ = insertion_cost_routes(G_comp_arr, vertices_tours,\
                                                                max_tour_vhcl_index, t, dep)
                
            # Modifying the target allocation array
            targ_allocation_arr_mod = copy.deepcopy(targ_allocation_arr)
            # Removing the considered target t from the vehicle with the maximal tour
            targ_allocation_arr_mod[max_tour_vhcl_index].remove(t)
            # Adding the considered target t to the vehicle with the minimum insertion
            # cost
            targ_allocation_arr_mod[min_insertion_vhcl_index].append(t)
            
            # print('Testing improvement.')
            # Resolving the tours for the vehicle with the maximum tour cost and
            # the vehicle with the minimum insertion cost
            obj_tours_two_vhcls, vertices_tours_two_vhcls =\
                MD_algorithm_constructing_tours_LKH(G_comp_arr, targ_allocation_arr_mod, dep,\
                                                    [max_tour_vhcl_index, min_insertion_vhcl_index])
                    
            # Checking if the newly obtained tours help reduce the maximum tour cost
            obj_tours_mod = copy.deepcopy(obj_tours)
            obj_tours_mod[max_tour_vhcl_index] = obj_tours_two_vhcls[0]
            obj_tours_mod[min_insertion_vhcl_index] = obj_tours_two_vhcls[1]
            
            # print('Modified target allocation array is ' + str(targ_allocation_arr_mod))
            # print('Objective obtained from trying to remove max target from LKH is ' +\
            #       str(obj_tours_mod) + ', whereas initial objective is ' + str(obj_tours))
            # print('New tour potentially obtained is ' + str(vertices_tours_two_vhcls))
            
            if max(obj_tours_mod) < max(obj_tours):
                
                obj_tours = obj_tours_mod
                targ_allocation_arr = targ_allocation_arr_mod
                vertices_tours[max_tour_vhcl_index] = vertices_tours_two_vhcls[0]
                vertices_tours[min_insertion_vhcl_index] = vertices_tours_two_vhcls[1]
                # print('Local improvement in solution obtained. New objective values are '\
                #       + str(obj_tours) + ', new target allocation is ' + str(targ_allocation_arr)\
                #       + ' and new tours are ' + str(vertices_tours) + '.')
                
                flag = 0 # Modifying the flag variable, since a local improvement
                # has been obtained
                
                break # Breaking from the loop in searching for replacing a target from
                # the maximum tour
                
    # If no more switch in targets helps, we look for target swap
    if flag == 0: # In this case, local improvement in solution was obtained
    
        flag = 1
        
    else: # In this case, no local improvement was previously made
    
        flag = 0
        
        print('No improvement in target switch made.')
        
    while flag != 1:
        
        flag = 1 # Changing the flag variable, which will then be set to zero
        # only if an improvement is obtained
        
        # Obtaining the vehicle index corresponding to the maximum cost tour
        max_tour_vhcl_index = obj_tours.index(max(obj_tours))
        
        # Obtaining the sorted list of target swaps along with vehicle index
        ranking_target_swap = neighbourhood_two_target_swap(G_comp_arr, vertices_tours,\
                                                            max_tour_vhcl_index, dep)
        # print(ranking_target_swap)
        # Running through the rankings, and checking for those wherein the value is
        # negative
        for (target_swap_vhcl, val) in ranking_target_swap:
            
            # Breaking the loop if value is negative
            # if val < 0:
                
            #     break
            
            # If not, we run LKH for the two vehicles. Note that "target_swap_vhcl"
            # is of the form (t1, t2, vhcl), wherein t1 is target to be removed from
            # maximal vehicle, t2 is target to be removed from other vehicle, and
            # vhcl is the index of the vehicle with which the exchange is done
            
            # Modifying the target allocation array
            targ_allocation_arr_mod = copy.deepcopy(targ_allocation_arr)
            # Removing the considered target t from the vehicle with the maximal tour
            targ_allocation_arr_mod[max_tour_vhcl_index].remove(target_swap_vhcl[0])
            targ_allocation_arr_mod[max_tour_vhcl_index].append(target_swap_vhcl[1])
            # Adding the considered target t to the vehicle with the minimum insertion
            # cost
            targ_allocation_arr_mod[target_swap_vhcl[2]].remove(target_swap_vhcl[1])
            targ_allocation_arr_mod[target_swap_vhcl[2]].append(target_swap_vhcl[0])
            
            # Resolving the tours for the vehicle with the maximum tour cost and
            # the vehicle with the minimum insertion cost
            obj_tours_two_vhcls, vertices_tours_two_vhcls =\
                MD_algorithm_constructing_tours_LKH(G_comp_arr, targ_allocation_arr_mod, dep,\
                                                    [max_tour_vhcl_index, target_swap_vhcl[2]])
                    
            # Checking if the newly obtained tours help reduce the maximum tour cost
            obj_tours_mod = copy.deepcopy(obj_tours)
            obj_tours_mod[max_tour_vhcl_index] = obj_tours_two_vhcls[0]
            obj_tours_mod[target_swap_vhcl[2]] = obj_tours_two_vhcls[1]
            
            # print('Modified target allocation array is ' + str(targ_allocation_arr_mod))
            # print('Objective obtained from trying to swap targets is ' +\
            #       str(obj_tours_mod) + ', whereas initial objective is ' + str(obj_tours))
            # print('New tour potentially obtained is ' + str(vertices_tours_two_vhcls))
            
            if max(obj_tours_mod) < max(obj_tours):
                
                obj_tours = obj_tours_mod
                targ_allocation_arr = targ_allocation_arr_mod
                vertices_tours[max_tour_vhcl_index] = vertices_tours_two_vhcls[0]
                vertices_tours[target_swap_vhcl[2]] = vertices_tours_two_vhcls[1]
                # print('Local improvement in solution obtained. New objective values are '\
                #       + str(obj_tours) + ', new target allocation is ' + str(targ_allocation_arr)\
                #       + ' and new tours are ' + str(vertices_tours) + '.')
                
                flag = 0 # Modifying the flag variable, since a local improvement
                # has been obtained
                
                break # Breaking from the loop in searching for replacing a target from
                # the maximum tour
                
    return obj_tours, targ_allocation_arr, vertices_tours

def perturbation_solution(G_comp_arr, T, targ_allocation_arr, dep, obj_tours,\
                          vertices_tours, mode = 1, angle_perturbations = False):
           
    # Generating five angles of perturbations for each depot
    rand_angle_perturb_depots = []
    
    # angle_perturbations = [15.642*math.pi/180, 76.490*math.pi/180, 220.360*math.pi/180]
    angle_perturbations_arr = []
    
    for i in range(len(G_comp_arr)):
        
        if angle_perturbations == False:
            
            rand_angle_perturb = np.random.rand()*2*math.pi
            angle_perturbations_arr.append(rand_angle_perturb)
            
        else:
            
            rand_angle_perturb = angle_perturbations[i]
        
        # Generating the other angles of perturbation by 144 deg using "rand_angle_perturb"
        # as the first angle of perturbation
        rand_angle_ith_depot_perturb = [np.mod((rand_angle_perturb + j*4*math.pi/5), 2*math.pi) for j in range(5)]
        
        # Appending to the array
        rand_angle_perturb_depots.append(rand_angle_ith_depot_perturb)
        
    # print('Angles of perturbation are ' + str(rand_angle_perturb_depots))
        
    # Running through each perturbation
    for i in range(5):
        
        # Using the current solution to compute average distances of each depot wrt
        # to the targets to which they are connected
        avg_distances = []
        
        for j in range(len(G_comp_arr)):
            
            # The targets to which the depot is connected to in the given solution are given
            # by vertices_tours[j][1] and vertices_tours[j][-2].
            avg_distance = (G_comp_arr[j][vertices_tours[j][1]][dep[j]]['weight']\
                            + G_comp_arr[j][vertices_tours[j][-2]][dep[j]]['weight'])/2
            avg_distances.append(avg_distance)
        
        # print('Average distances are ' + str(avg_distances))
        # Creating a new array of graphs with changed depot location
        G_comp_arr_new = []
        for j in range(len(G_comp_arr)):
            
            temp = nx.Graph()
            G_comp_arr_new.append(temp)
            
        # Adding targets to all the graphs
        position = nx.get_node_attributes(G_comp_arr[0], 'pos')
        
        for t in T:
            
            for j in range(len(G_comp_arr)):
                
                G_comp_arr_new[j].add_node(t, pos = position[t])
                
        # Adding the depot for each vehicle in the displaced position
        for j in range(len(G_comp_arr)):
            
            position = nx.get_node_attributes(G_comp_arr[j], 'pos')
            
            # print('Angle of perturbation of vehicle ' + str(j + 1) + ' is ' +\
            #       str(rand_angle_perturb_depots[j][i]))
            
            # Obtaining the new position of the depot for the jth vehicle, which
            # is the initial position vector + (rcos(theta), rsin(theta)). Here,
            # r is the average distance of the chosen vehicle's depot from the
            # targets to which it is attached, and theta denotes the angle of
            # perturbation for the jth vehicle for the ith perturbation number.
            new_pos = (position[dep[j]][0] + avg_distances[j]*math.cos(rand_angle_perturb_depots[j][i]),\
                       position[dep[j]][1] + avg_distances[j]*math.sin(rand_angle_perturb_depots[j][i]))
            
            # Adding the depot with the new location to the graph
            G_comp_arr_new[j].add_node(dep[j], pos = new_pos)
            
        # Adding edges to each graph
        # Adding edges between targets
        for (t1, t2) in combinations(T, 2):
            
            for j in range(len(G_comp_arr)):
                
                G_comp_arr_new[j].add_edge(t1, t2, weight = G_comp_arr[0][t1][t2]['weight'])
                
        # Adding edges between a target and the depot for a vehicle
        for j in range(len(G_comp_arr_new)):
            
            # Obtaining the position of all nodes
            position = nx.get_node_attributes(G_comp_arr_new[j], 'pos')
            
            # Running through all targets
            for t in T:
                
                # Computing the distance between the target and the depot
                dist = math.sqrt((position[t][0] - position[dep[j]][0])**2 +\
                                 (position[t][1] - position[dep[j]][1])**2)
                
                # Adding the edge
                G_comp_arr_new[j].add_edge(t, dep[j], weight = dist)
                
        # Utilizing the obtained target allocation after local search as an initial 
        # feasible solution after perturbation
        obj_tours_perturb_ini, vertices_tours_perturb_ini =\
            MD_algorithm_constructing_tours_LKH(G_comp_arr_new, targ_allocation_arr, dep,\
                                                [i for i in range(len(G_comp_arr_new))])
           
        # print('The initial tour costs for perturbed depots using same initial targ allocation ' +\
        #       ' is ' + str(obj_tours_perturb_ini))
            
        # Performing a local search using the obtained tours
        if mode == 1:
            
            obj_tours_perturb_loc, targ_allocation_arr_loc, vertices_tours_loc =\
                local_search_maximal_route(G_comp_arr_new, targ_allocation_arr, dep,\
                                           obj_tours_perturb_ini, vertices_tours_perturb_ini)
                    
        elif mode == 2:
            
            obj_tours_perturb_loc, targ_allocation_arr_loc, vertices_tours_loc =\
                local_search_maximal_route_switch_and_swap(G_comp_arr_new, targ_allocation_arr, dep,\
                                                           obj_tours_perturb_ini, vertices_tours_perturb_ini)
                
        # print('Now, the solution obtained after the local search is used for original graph.')
        
        # Using the obtained target allocation to construct feasible solution for
        # the original graphs
        obj_tours_perturb_i, vertices_tours_perturb_i =\
            MD_algorithm_constructing_tours_LKH(G_comp_arr, targ_allocation_arr_loc, dep,\
                                                [i for i in range(len(G_comp_arr))])
                
        # print('The initial tour costs obtained from the perturbed depots for original ' +\
        #       'graph is ' + str(obj_tours_perturb_i))
        # print('Initial tours are ' + str(vertices_tours_perturb_i))
                
        # print('Performing local search on the original graph.')
                
        # Performing another local search
        obj_tours_perturb_fin, targ_allocation_arr_fin, vertices_tours_fin =\
            local_search_maximal_route(G_comp_arr, targ_allocation_arr_loc, dep,\
                                       obj_tours_perturb_i, vertices_tours_perturb_i)
                
        # Checking if the solution has improved
        if max(obj_tours_perturb_fin) < max(obj_tours):
            
            # print('Better solution obtained after perturbation.')
            obj_tours = obj_tours_perturb_fin
            targ_allocation_arr = targ_allocation_arr_fin
            vertices_tours = vertices_tours_fin
                
    return obj_tours, targ_allocation_arr, vertices_tours, angle_perturbations_arr

def neighbourhood_two_target_swap(G_comp_arr, vertices_tours, max_tour_vhcl_index, dep):
    '''
    In this function, neighbourhoods are explored wherein a target in the maximal
    vehicle's tour is swapped with a target of another vehicle. For this purpose,
    a ranking system is constructed based on a metric.

    Parameters
    ----------
    G_comp_arr : Array
        Contains the graph for each vehicle.
    vertices_tours : Array
        Contains the vertices in the tours for each vehicle.
    max_tour_vhcl_index : Scalar
        Describes the vehicle that has the maximum tour cost.

    Returns
    -------
    ranking_target_vehicle_swap : Array
        Contains the ranking of target swap.

    '''
    
    # Obtaining the sorted savings for the maximal vehicle
    savings_maximal_vehicle = savings_removing_target_max_route(G_comp_arr[max_tour_vhcl_index],\
                                                                vertices_tours[max_tour_vhcl_index])
        
    # Creating a dictionary to keep track of target swap and which vehicle the
    # targets are being swapped with
    targ_swap = {}    
    
    # Running through the other vehicles to obtain cost of target swap
    for i in range(len(G_comp_arr)):
        
        # Checking if the chosen vehicle is the same as the maximal vehicle
        if i == max_tour_vhcl_index:
            
            continue
        
        # Obtaining the savings for the chosen vehicle
        savings_chosen_vehicle = savings_removing_target_max_route(G_comp_arr[i],\
                                                                   vertices_tours[i])
    
        # Running through each target in the maximal tour and target in the other
        # vehicle's tour
        for (j, val1) in savings_maximal_vehicle:
            
            for (k, val2) in savings_chosen_vehicle:
                
                # Obtaining the cost of adding target k to the maximal vehicle,
                # and cost of adding target j to the chosen vehicle
                # For this purpose, we first modify the tours of the two vehicles
                # as target j and target k have been removed from the maximal vehicle
                # and the chosen vehicle, respectively.
                tour_maximal_vehicle = copy.deepcopy(vertices_tours[max_tour_vhcl_index])
                tour_maximal_vehicle.remove(j)
                
                tour_chosen_vehicle = copy.deepcopy(vertices_tours[i])
                tour_chosen_vehicle.remove(k)
                
                _, cost_increase_maximal_vehicle =\
                    insertion_cost_routes([G_comp_arr[max_tour_vhcl_index]], [tour_maximal_vehicle],\
                                          np.NaN, k, dep)
                _, cost_increase_chosen_vehicle =\
                    insertion_cost_routes([G_comp_arr[i]], [tour_chosen_vehicle],\
                                          np.NaN, j, dep)
                
                # Computing the defined metric
                metric_target_swap_chosen = cost_increase_maximal_vehicle + cost_increase_chosen_vehicle\
                    - val1 - val2
                # Checking if for the maximal vehicle, the tour cost can be expected
                # to decrease, and the net decrease is negative
                # if cost_increase_maximal_vehicle - val1 < 0 and metric_target_swap_chosen < 0:
                if cost_increase_maximal_vehicle - val1 < 0:
                    
                    # Appending to the dictionary
                    targ_swap[(j, k, i)] = metric_target_swap_chosen
                
    # Sorting the target swap in increasing order, since we want the least cost
    # of increase and highest savings
    ranking_target_vehicle_swap = sorted(targ_swap.items(), key = lambda x:x[1])
    
    return ranking_target_vehicle_swap