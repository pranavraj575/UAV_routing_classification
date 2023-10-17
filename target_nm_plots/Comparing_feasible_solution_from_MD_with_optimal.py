# -*- coding: utf-8 -*-
"""
Created on Fri Apr  7 09:47:01 2023

@author: deepa
"""

import numpy as np
import networkx as nx
import os
from itertools import combinations
import math
import matplotlib.pyplot as plt
import time
import pandas as pd

path = 'D:\TAMU\Research\Min-max TSP\Codes\Min-max depot symmetric TSP branch and cut'
os.chdir(path)

# Importing the functions for the MD algorithm and the MILP
from Function_initial_feasible_solution_MD_algorithm import MD_algorithm
from Multi_vehicle_min_max_sym_depot_TSP_branch_and_cut_LP_cutting_plane\
    import multi_vhcl_sym_TSP_fixed_depot
# Importing function for generating a random graph
from Random_instance_generation_comparison_bounds import random_graph_generation_sym_TSP_depots
    
def testing_MD_algorithm_given_data():
    
    # # Defining the coordinates of the customers
    # coords = {1: (10, 100), 2: (20, 40), 3: (70, 80), 4: (30, 40),\
    #           5: (20, 50), 6: (90, 30), 7: (50, 70), 8: (80, 30),\
    #           9: (30, 70), 10: (80, 90)}
        
    # T = [(i + 1) for i in range(10)]
    # num_vhcls = 3
        
    # # Defining the coordinates of the depots and terminals
    # dep_loc = {11: (10, 20), 13: (20, 30), 15: (30, 10)}
    # term_loc = {12: (10, 20), 14: (20, 30), 16: (30, 10)}
    
    # dep = [11, 13, 15]
    # term = [12, 14, 16]
    
    # Defining the coordinates of the customers
    coords = {1: (48.38902148, 102.6605505), 2: (110.6801909, 12.3853211),\
              3: (172.0167064, 8.348623853), 4: (138.1264916, 121.7431193),\
              5: (183.9498807, 160.6422018), 6: (82.51789976, 33.30275229),\
              7: (174.1646778, 27.79816514), 8: (21.8973747, 2.47706422),\
              9: (189.6778043, 137.5229358), 10: (93.73508353, 65.59633028)}
        
    T = [(i + 1) for i in range(10)]
    num_vhcls = 2
        
    # Defining the coordinates of the depots and terminals
    dep_loc = {11: (67.24343675, 138.9908257), 13: (119.0334129, 155.5045872)}
    term_loc = {12: (67.24343675, 138.9908257), 14: (119.0334129, 155.5045872)}
    
    dep = [11, 13]
    term = [12, 14]
    
    # Constructing the graph for each vehicle
    G_comp_arr = []
    for i in range(num_vhcls):
    
        temp = nx.Graph()
        G_comp_arr.append(temp)
    
    # Adding targets to the graphs
    for i in T:
        
        for j in range(num_vhcls):
            
            G_comp_arr[j].add_node(i, pos = coords[i])
            
    # Adding depots and terminals to the corresponding graphs
    for i in range(num_vhcls):
        
        G_comp_arr[i].add_node(dep[i], pos = dep_loc[dep[i]])
        G_comp_arr[i].add_node(term[i], pos = term_loc[term[i]])
        
    # Adding edges to the graphs
    pos = nx.get_node_attributes(G_comp_arr[0], 'pos') # Obtaining coords of all nodes
    
    # Running through all targets and using the distance between them as the
    # edge weight
    for (i, j) in list(combinations(T, 2)):
        
        # Obtaining the distance between the two nodes
        dist = math.sqrt((pos[j][0] - pos[i][0])**2 + (pos[j][1] - pos[i][1])**2)
        
        # Adding the edge to all graphs
        for k in range(num_vhcls):
            
            G_comp_arr[k].add_edge(i, j, weight = dist)
            
    # Constructing the edges from targets to the depot and the terminal for each vehicle
    for i in range(num_vhcls):
        
        # Obtaining coords of all nodes for the (i + 1)th vehicle
        pos = nx.get_node_attributes(G_comp_arr[i], 'pos')
        
        for j in T:
            
            dist =  math.sqrt((pos[dep[i]][0] - pos[j][0])**2 + (pos[dep[i]][1] - pos[j][1])**2)
            G_comp_arr[i].add_edge(j, dep[i], weight = dist)
            G_comp_arr[i].add_edge(j, term[i], weight = dist) # As depot and terminal are at
            # the same coordinates
            
    # Finally, adding a zero cost edge between a depot and its corresponding terminal
    for i in range(num_vhcls):
        
        G_comp_arr[i].add_edge(dep[i], term[i], weight = 0)    
            
    return G_comp_arr, T, dep, term

def comparison_random_instances(num_vhcls, num_T, l_region_targets, num_instances):
    '''
    In this function, random instances are generated. The optimal solution is compared
    with the solution obtained from the MD algoritm.

    Parameters
    ----------
    num_vhcls : Scalar
        Number of vehicles.
    num_T : Scalar
        Number of targets.
    l_region_targets : Scalar
        Dimension (length) of square in which the targets and depots are generated.
    num_instances : Scalar
        Number of instances for the comparison.

    Returns
    -------
    None.

    '''
    
    # Initializing arrays to compare the optimal solution with the solution from
    # the MD algorithm (without and with swap of targets)
    optimal_cost = []
    MD_obj_cost = []
    MD_obj_loc_search_cost = []
    MD_obj_cost_swap = []
    MD_obj_loc_search_cost_swap = []
    optimal_runtime = []
    MD_runtime = []
    MD_runtime_swap = []
    
    # Keeping track of vehicle tour costs per vehicle for MD algorithm without swap
    MD_obj_cost_per_vhcl = []
    MD_obj_loc_search_cost_per_vhcl = []
    # Keeping track of vehicle tour costs per vehicle for MD algorithm with swap
    MD_obj_cost_per_vhcl_swap = []
    MD_obj_loc_search_cost_per_vhcl_swap = []
    
    for i in range(num_vhcls):
        
        MD_obj_cost_per_vhcl.append([])
        MD_obj_loc_search_cost_per_vhcl.append([])
        MD_obj_cost_per_vhcl_swap.append([])
        MD_obj_loc_search_cost_per_vhcl_swap.append([])
    
    for n in range(num_instances):
        
        # Generating a random graph
        G_comp_arr, T, dep, term = random_graph_generation_sym_TSP_depots(num_vhcls, num_T, l_region_targets)
        # G_comp_arr, T, dep, term = testing_MD_algorithm_given_data()
        
        # Computing the optimal solution
        obj, x_vals, phi_vals, bound, runtime, lb, ub =\
            multi_vhcl_sym_TSP_fixed_depot(G_comp_arr, T, dep, term, [], 'IP_subtour', 'MD')
            
        optimal_runtime.append(runtime)    
        
        # Obtaining the solution from the MD algorithm (without target swap)
        start = time.time()
        obj_tours, _, tours, obj_ini, obj_loc_search, _, tours_loc, angle_perturbations\
            = MD_algorithm(G_comp_arr, T, dep, 1, False)
        end = time.time()
        MD_runtime.append(end - start)
        
        # Saving the objective function value
        optimal_cost.append(obj[0])
        MD_obj_cost.append(max(obj_tours))
        MD_obj_loc_search_cost.append(max(obj_loc_search))
        
        # Keeping track of tour cost for each vehicle
        for i in range(num_vhcls):
            
            MD_obj_cost_per_vhcl[i].append(obj_tours[i])
            MD_obj_loc_search_cost_per_vhcl[i].append(obj_loc_search[i])
            
        # Obtaining the solution from the MD algorithm with target swap (using the same
        # angles of perturbation)
        start = time.time()
        obj_tours, _, tours_swap, obj_ini, obj_loc_search, _, tours_loc, _ =\
            MD_algorithm(G_comp_arr, T, dep, 2, angle_perturbations)
        end = time.time()
        MD_runtime_swap.append(end - start)
        
        # Saving the objective function value
        MD_obj_cost_swap.append(max(obj_tours))
        MD_obj_loc_search_cost_swap.append(max(obj_loc_search))
        
        # Plotting the tours
        plotting_optimal_solution_MD_solution(G_comp_arr, T, dep, term,\
                                              l_region_targets, tours, tours_swap, x_vals)
            
        # Keeping track of tour cost for each vehicle
        for i in range(num_vhcls):
            
            MD_obj_cost_per_vhcl_swap[i].append(obj_tours[i])
            MD_obj_loc_search_cost_per_vhcl_swap[i].append(obj_loc_search[i])
        
    # Plotting the comparison of the optimal solution and the MD algorithm
    instance_nos = [(i + 1) for i in range(num_instances)]
    dev = [((MD_obj_cost[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    dev_loc_search = [((MD_obj_loc_search_cost[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    dev_swap = [((MD_obj_cost_swap[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    dev_loc_search_swap = [((MD_obj_loc_search_cost_swap[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    
    plt.figure()
    plt.plot(instance_nos, dev_loc_search, 'r+', linewidth = 1.5, label = 'MD algo local search')
    plt.plot(instance_nos, dev, 'bo', linewidth = 1.5, label = 'MD algo perturbation', fillstyle = None)
    plt.plot(instance_nos, dev_loc_search_swap, 'gx', linewidth = 1.5,\
             label = 'MD algo (swap) local search', fillstyle = None)
    plt.plot(instance_nos, dev_swap, 'kv', linewidth = 1.5, label = 'MD algo (swap) perturbation')
    plt.xlabel('Instance number')
    plt.ylabel('Percentage deviation of MD solution from optimum')
    plt.title('Percentage deviation for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()
    
    # Plotting the tour cost of each vehicle
    plt.figure()
    marker_style = ['r+', 'b*', 'gd', 'kx']
    for i in range(num_vhcls):
        
        plt.plot(instance_nos, MD_obj_cost_per_vhcl[i], marker_style[i], linewidth = 1.5,\
                 label = 'Vehicle ' + str(i + 1))
    
    plt.xlabel('Instance number')
    plt.ylabel('Tour cost')
    plt.title('Tour costs of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()
    
    # Plotting the tour cost of each vehicle with MD swap
    plt.figure()
    marker_style = ['r+', 'b*', 'gd', 'kx']
    for i in range(num_vhcls):
        
        plt.plot(instance_nos, MD_obj_cost_per_vhcl_swap[i], marker_style[i], linewidth = 1.5,\
                 label = 'Vehicle ' + str(i + 1))
    
    plt.xlabel('Instance number')
    plt.ylabel('Tour cost')
    plt.title('Tour costs of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets (swap)')
    plt.legend()
    plt.show()
    
    # Plotting the runtimes
    plt.figure()
    plt.plot(instance_nos, optimal_runtime, 'r+', linewidth = 1.5, label = 'Optimum')
    plt.plot(instance_nos, MD_runtime, 'b*', linewidth = 1.5, label = 'MD algorithm')
    plt.plot(instance_nos, MD_runtime_swap, 'gd', linewidth = 1.5, label = 'MD algorithm (swap)')
    plt.xlabel('Instance number')
    plt.ylabel('Runtime (s)')
    plt.title('Runtime of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()    
    
    # Plotting the runtimes for the two implementation of the MD algorithm
    plt.figure()
    plt.plot(instance_nos, MD_runtime, 'b*', linewidth = 1.5, label = 'MD algorithm')
    plt.plot(instance_nos, MD_runtime_swap, 'gd', linewidth = 1.5, label = 'MD algorithm (swap)')
    plt.xlabel('Instance number')
    plt.ylabel('Runtime (s)')
    plt.title('Runtime of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show() 
    
    return dev, dev_loc_search, dev_swap, dev_loc_search_swap, optimal_runtime, MD_runtime, MD_runtime_swap

def comparison_random_instances_no_optimal(num_vhcls, num_T, l_region_targets, num_instances):
    '''
    In this function, random instances are generated. The optimal solution is compared
    with the solution obtained from the MD algoritm.

    Parameters
    ----------
    num_vhcls : Scalar
        Number of vehicles.
    num_T : Scalar
        Number of targets.
    l_region_targets : Scalar
        Dimension (length) of square in which the targets and depots are generated.
    num_instances : Scalar
        Number of instances for the comparison.

    Returns
    -------
    None.

    '''
    
    # Initializing arrays to compare the optimal solution with the solution from
    # the MD algorithm (without and with swap of targets)
    MD_obj_cost = []
    MD_obj_loc_search_cost = []
    MD_obj_cost_swap = []
    MD_obj_loc_search_cost_swap = []
    MD_runtime = []
    MD_runtime_swap = []
    
    # Keeping track of vehicle tour costs per vehicle for MD algorithm without swap
    MD_obj_cost_per_vhcl = []
    MD_obj_loc_search_cost_per_vhcl = []
    # Keeping track of vehicle tour costs per vehicle for MD algorithm with swap
    MD_obj_cost_per_vhcl_swap = []
    MD_obj_loc_search_cost_per_vhcl_swap = []
    
    for i in range(num_vhcls):
        
        MD_obj_cost_per_vhcl.append([])
        MD_obj_loc_search_cost_per_vhcl.append([])
        MD_obj_cost_per_vhcl_swap.append([])
        MD_obj_loc_search_cost_per_vhcl_swap.append([])
    
    for n in range(num_instances):
        
        # Generating a random graph
        G_comp_arr, T, dep, term = random_graph_generation_sym_TSP_depots(num_vhcls, num_T, l_region_targets)
        # G_comp_arr, T, dep, term = testing_MD_algorithm_given_data() 
        
        # Obtaining the solution from the MD algorithm (without target swap)
        start = time.time()
        obj_tours, _, tours, obj_ini, obj_loc_search, _, tours_loc, angle_perturbations\
            = MD_algorithm(G_comp_arr, T, dep, 1, False)
        end = time.time()
        MD_runtime.append(end - start)
        
        # Saving the objective function value
        MD_obj_cost.append(max(obj_tours))
        MD_obj_loc_search_cost.append(max(obj_loc_search))
        
        # Keeping track of tour cost for each vehicle
        for i in range(num_vhcls):
            
            MD_obj_cost_per_vhcl[i].append(obj_tours[i])
            MD_obj_loc_search_cost_per_vhcl[i].append(obj_loc_search[i])
            
        # Obtaining the solution from the MD algorithm with target swap (using the same
        # angles of perturbation)
        start = time.time()
        obj_tours, _, tours_swap, obj_ini, obj_loc_search, _, tours_loc, _ =\
            MD_algorithm(G_comp_arr, T, dep, 2, angle_perturbations)
        end = time.time()
        MD_runtime_swap.append(end - start)
        
        # Saving the objective function value
        MD_obj_cost_swap.append(max(obj_tours))
        MD_obj_loc_search_cost_swap.append(max(obj_loc_search))
        
        # Plotting the tours
        # plotting_optimal_solution_MD_solution(G_comp_arr, T, dep, term,\
        #                                       l_region_targets, tours, tours_swap, x_vals)
            
        # Keeping track of tour cost for each vehicle
        for i in range(num_vhcls):
            
            MD_obj_cost_per_vhcl_swap[i].append(obj_tours[i])
            MD_obj_loc_search_cost_per_vhcl_swap[i].append(obj_loc_search[i])
        
    # Plotting the comparison of the optimal solution and the MD algorithm
    instance_nos = [(i + 1) for i in range(num_instances)]
    # dev = [((MD_obj_cost[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    # dev_loc_search = [((MD_obj_loc_search_cost[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    # dev_swap = [((MD_obj_cost_swap[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    # dev_loc_search_swap = [((MD_obj_loc_search_cost_swap[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    dev = [(MD_obj_cost_swap[i] - MD_obj_cost[i])/MD_obj_cost[i]*100 for i in range(num_instances)]
    
    plt.figure()
    plt.plot(instance_nos, dev, 'bo', linewidth = 1.5, fillstyle = None)
    plt.xlabel('Instance number')
    plt.ylabel('Percentage deviation of MD solution with swap from without swap')
    plt.title('Percentage deviation for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()
    
    # Plotting the tour cost of each vehicle
    plt.figure()
    marker_style = ['r+', 'b*', 'gd', 'kx']
    for i in range(num_vhcls):
        
        plt.plot(instance_nos, MD_obj_cost_per_vhcl[i], marker_style[i], linewidth = 1.5,\
                 label = 'Vehicle ' + str(i + 1))
    
    plt.xlabel('Instance number')
    plt.ylabel('Tour cost')
    plt.title('Tour costs of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()
    
    # Plotting the tour cost of each vehicle with MD swap
    plt.figure()
    marker_style = ['r+', 'b*', 'gd', 'kx']
    for i in range(num_vhcls):
        
        plt.plot(instance_nos, MD_obj_cost_per_vhcl_swap[i], marker_style[i], linewidth = 1.5,\
                 label = 'Vehicle ' + str(i + 1))
    
    plt.xlabel('Instance number')
    plt.ylabel('Tour cost')
    plt.title('Tour costs of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets (swap)')
    plt.legend()
    plt.show()
    
    # Plotting the runtimes
    plt.figure()
    plt.plot(instance_nos, MD_runtime, 'b*', linewidth = 1.5, label = 'MD algorithm')
    plt.plot(instance_nos, MD_runtime_swap, 'gd', linewidth = 1.5, label = 'MD algorithm (swap)')
    plt.xlabel('Instance number')
    plt.ylabel('Runtime (s)')
    plt.title('Runtime of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()    
        
    return dev, MD_runtime, MD_runtime_swap

def plotting_optimal_solution_MD_solution(G_comp_arr, T, dep, term, l_region_targets,\
                                          tours, tours_MD_swap, x_vals):
    '''
    In this function, the results from the MD algorithm and the optimal solution
    are plotted

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
    l_region_targets : TYPE
        DESCRIPTION.
    tours : TYPE
        DESCRIPTION.
    x_vals : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    # Creating an array for plot colors
    color = ['r', 'b', 'g', 'k', 'brown', 'cyan']
    
    fig, ax = plt.subplots(1, 3, figsize = (10, 4.8))
    
    # Plotting the nodes
    G = nx.Graph()
    position = nx.get_node_attributes(G_comp_arr[0], 'pos')
    
    color_map = []
    for n in G_comp_arr[0].nodes():
        
        if n not in dep and n not in term:
            
            G.add_node(n, pos = position[n])
            color_map.append('#1f78b4')
            
    # Adding the depot for the first vehicle
    G.add_node(dep[0], pos = position[dep[0]])
    color_map.append('g')
            
    # Adding depots of other vehicles
    for j in range(1, len(dep)):
        
        position = nx.get_node_attributes(G_comp_arr[j], 'pos')
        G.add_node(dep[j], pos = position[dep[j]])
        color_map.append('g')
        
    # Plotting the nodes
    position = nx.get_node_attributes(G, 'pos')
    
    for i in range(3):
        
        nx.draw_networkx_nodes(G, pos = position, ax = ax[i], node_color = color_map)
        nx.draw_networkx_labels(G, position, ax = ax[i])
        
    # Plotting the solution from the MD algorithm
    for i in range(len(dep)):
        
        G = nx.Graph()
        position = nx.get_node_attributes(G_comp_arr[i], 'pos')
        
        for j in tours[i]:
            
            G.add_node(j, pos = position[j])
            
        # Adding the edges from the tour
        for j in range(1, len(tours[i])):
            
            G.add_edge(tours[i][j - 1], tours[i][j])
        
        position = nx.get_node_attributes(G, 'pos')
        # Plotting
        nx.draw_networkx_edges(G, pos = position, edge_color = color[i], width = 2,\
                               label = 'Vehicle ' + str(i + 1), ax = ax[0])
            
    # Adding labels for the plot
    ax[0].set_axis_on()
    ax[0].tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    ax[0].set_xlabel('X (m)')
    ax[0].set_ylabel('Y (m)')
    ax[0].legend()
    ax[0].set_title('Solution from MD algorithm')
    
    # Plotting the solution from the MD algorithm with target swap
    for i in range(len(dep)):
        
        G = nx.Graph()
        position = nx.get_node_attributes(G_comp_arr[i], 'pos')
        
        for j in tours_MD_swap[i]:
            
            G.add_node(j, pos = position[j])
            
        # Adding the edges from the tour
        for j in range(1, len(tours_MD_swap[i])):
            
            G.add_edge(tours_MD_swap[i][j - 1], tours_MD_swap[i][j])
        
        position = nx.get_node_attributes(G, 'pos')
        # Plotting
        nx.draw_networkx_edges(G, pos = position, edge_color = color[i], width = 2,\
                               label = 'Vehicle ' + str(i + 1), ax = ax[1])
            
    # Adding labels for the plot
    ax[1].set_axis_on()
    ax[1].tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    ax[1].set_xlabel('X (m)')
    ax[1].set_ylabel('Y (m)')
    ax[1].legend()
    ax[1].set_title('Solution from MD algorithm (swap)')
    
    # Plotting the optimal solution
    for j in range(len(dep)):
        
        # Constructing the graph corresponding to the solution for vehicle i
        G = nx.Graph()
        
        # Adding the nodes to the graph
        position = nx.get_node_attributes(G_comp_arr[j], 'pos')
        
        for i in G_comp_arr[j].nodes():
            
            if i not in term:
            
                G.add_node(i, pos = position[i])
        
        for e, val in x_vals[j].items():
            
            if val >= 0.5:
                
                if (e[0] in term and e[1] in dep) or (e[0] in dep and e[1] in term):
                    
                    continue
                
                elif e[0] in term:
                    
                    G.add_edge(dep[j], e[1])
                
                elif e[1] in term:
                    
                    G.add_edge(e[0], dep[j])
                    
                else:
                
                    G.add_edge(e[0], e[1])
                    
        nx.draw_networkx_edges(G, pos = position, edge_color = color[j], width = 2,\
                               label = 'Vehicle ' + str(j + 1), ax = ax[2])
    # Adding labels for the plot
    ax[2].set_axis_on()
    ax[2].tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    ax[2].set_xlabel('X (m)')
    ax[2].set_ylabel('Y (m)')
    ax[2].legend()
    ax[2].set_title('Optimal solution')
    
def comparison_random_instances_MD(num_vhcls, num_T, l_region_targets, num_instances):
    '''
    In this function, random instances are generated. The optimal solution is compared
    with the solution obtained from the MD algoritm.

    Parameters
    ----------
    num_vhcls : Scalar
        Number of vehicles.
    num_T : Scalar
        Number of targets.
    l_region_targets : Scalar
        Dimension (length) of square in which the targets and depots are generated.
    num_instances : Scalar
        Number of instances for the comparison.

    Returns
    -------
    None.

    '''
    
    # Initializing arrays to compare the optimal solution with the solution from
    # the MD algorithm (without and with swap of targets)
    optimal_cost = []
    MD_obj_cost = []
    MD_obj_loc_search_cost = []
    optimal_runtime = []
    MD_runtime = []
    
    # Keeping track of vehicle tour costs per vehicle for MD algorithm without swap
    MD_obj_cost_per_vhcl = []
    MD_obj_loc_search_cost_per_vhcl = []
    
    for i in range(num_vhcls):
        
        MD_obj_cost_per_vhcl.append([])
        MD_obj_loc_search_cost_per_vhcl.append([])
    
    for n in range(num_instances):
        
        # Generating a random graph
        G_comp_arr, T, dep, term = random_graph_generation_sym_TSP_depots(num_vhcls, num_T, l_region_targets)
        # G_comp_arr, T, dep, term = testing_MD_algorithm_given_data()
        
        # Computing the optimal solution
        obj, x_vals, phi_vals, bound, runtime, lb, ub =\
            multi_vhcl_sym_TSP_fixed_depot(G_comp_arr, T, dep, term, [], 'IP_subtour', 'MD')
            
        optimal_runtime.append(runtime)    
        
        # Obtaining the solution from the MD algorithm (without target swap)
        start = time.time()
        obj_tours, _, tours, obj_ini, obj_loc_search, _, tours_loc, angle_perturbations\
            = MD_algorithm(G_comp_arr, T, dep, 1, False)
        end = time.time()
        MD_runtime.append(end - start)
        
        # Saving the objective function value
        optimal_cost.append(obj[0])
        MD_obj_cost.append(max(obj_tours))
        MD_obj_loc_search_cost.append(max(obj_loc_search))
        
        # Keeping track of tour cost for each vehicle
        for i in range(num_vhcls):
            
            MD_obj_cost_per_vhcl[i].append(obj_tours[i])
            MD_obj_loc_search_cost_per_vhcl[i].append(obj_loc_search[i])
        
        # Plotting the tours
        plotting_optimal_solution_MD_solution_no_swap(G_comp_arr, T, dep, term,\
                                                      l_region_targets, tours, x_vals)
        
    # Plotting the comparison of the optimal solution and the MD algorithm
    instance_nos = [(i + 1) for i in range(num_instances)]
    dev = [((MD_obj_cost[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    dev_loc_search = [((MD_obj_loc_search_cost[i] - optimal_cost[i])/optimal_cost[i])*100 for i in range(num_instances)]
    
    plt.figure()
    plt.plot(instance_nos, dev_loc_search, 'r+', linewidth = 1.5, label = 'MD algo local search')
    plt.plot(instance_nos, dev, 'bo', linewidth = 1.5, label = 'MD algo perturbation', fillstyle = None)
    plt.xlabel('Instance number')
    plt.ylabel('Percentage deviation of MD solution from optimum')
    plt.title('Percentage deviation for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()
    
    # Plotting the tour cost of each vehicle
    plt.figure()
    marker_style = ['r+', 'b*', 'gd', 'kx']
    for i in range(num_vhcls):
        
        plt.plot(instance_nos, MD_obj_cost_per_vhcl[i], marker_style[i], linewidth = 1.5,\
                 label = 'Vehicle ' + str(i + 1))
    
    plt.xlabel('Instance number')
    plt.ylabel('Tour cost')
    plt.title('Tour costs of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()
    
    # Plotting the runtimes
    plt.figure()
    plt.plot(instance_nos, optimal_runtime, 'r+', linewidth = 1.5, label = 'Optimum')
    plt.plot(instance_nos, MD_runtime, 'b*', linewidth = 1.5, label = 'MD algorithm')
    plt.xlabel('Instance number')
    plt.ylabel('Runtime (s)')
    plt.title('Runtime of vehicles for ' + str(num_vhcls) + ' vhcls, '\
              + str(num_T) + ' targets')
    plt.legend()
    plt.show()
    
    return optimal_cost, MD_obj_loc_search_cost, MD_obj_cost

def plotting_optimal_solution_MD_solution_no_swap(G_comp_arr, T, dep, term, l_region_targets,\
                                                  tours, x_vals):
    '''
    In this function, the results from the MD algorithm and the optimal solution
    are plotted

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
    l_region_targets : TYPE
        DESCRIPTION.
    tours : TYPE
        DESCRIPTION.
    x_vals : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    # Creating an array for plot colors
    color = ['r', 'b', 'g', 'k', 'brown', 'cyan']
    
    fig, ax = plt.subplots(1, 2, figsize = (10, 4.8))
    
    # Plotting the nodes
    G = nx.Graph()
    position = nx.get_node_attributes(G_comp_arr[0], 'pos')
    
    color_map = []
    for n in G_comp_arr[0].nodes():
        
        if n not in dep and n not in term:
            
            G.add_node(n, pos = position[n])
            color_map.append('#1f78b4')
            
    # Adding the depot for the first vehicle
    G.add_node(dep[0], pos = position[dep[0]])
    color_map.append('g')
            
    # Adding depots of other vehicles
    for j in range(1, len(dep)):
        
        position = nx.get_node_attributes(G_comp_arr[j], 'pos')
        G.add_node(dep[j], pos = position[dep[j]])
        color_map.append('g')
        
    # Plotting the nodes
    position = nx.get_node_attributes(G, 'pos')
    
    for i in range(2):
        
        nx.draw_networkx_nodes(G, pos = position, ax = ax[i], node_color = color_map)
        nx.draw_networkx_labels(G, position, ax = ax[i])
        
    # Plotting the solution from the MD algorithm
    for i in range(len(dep)):
        
        G = nx.Graph()
        position = nx.get_node_attributes(G_comp_arr[i], 'pos')
        
        for j in tours[i]:
            
            G.add_node(j, pos = position[j])
            
        # Adding the edges from the tour
        for j in range(1, len(tours[i])):
            
            G.add_edge(tours[i][j - 1], tours[i][j])
        
        position = nx.get_node_attributes(G, 'pos')
        # Plotting
        nx.draw_networkx_edges(G, pos = position, edge_color = color[i], width = 2,\
                               label = 'Vehicle ' + str(i + 1), ax = ax[0])
            
    # Adding labels for the plot
    ax[0].set_axis_on()
    ax[0].tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    ax[0].set_xlabel('X (m)')
    ax[0].set_ylabel('Y (m)')
    ax[0].legend()
    ax[0].set_title('Solution from MD algorithm')
    
    # Plotting the optimal solution
    for j in range(len(dep)):
        
        # Constructing the graph corresponding to the solution for vehicle i
        G = nx.Graph()
        
        # Adding the nodes to the graph
        position = nx.get_node_attributes(G_comp_arr[j], 'pos')
        
        for i in G_comp_arr[j].nodes():
            
            if i not in term:
            
                G.add_node(i, pos = position[i])
        
        for e, val in x_vals[j].items():
            
            if val >= 0.5:
                
                if (e[0] in term and e[1] in dep) or (e[0] in dep and e[1] in term):
                    
                    continue
                
                elif e[0] in term:
                    
                    G.add_edge(dep[j], e[1])
                
                elif e[1] in term:
                    
                    G.add_edge(e[0], dep[j])
                    
                else:
                
                    G.add_edge(e[0], e[1])
                    
        nx.draw_networkx_edges(G, pos = position, edge_color = color[j], width = 2,\
                               label = 'Vehicle ' + str(j + 1), ax = ax[1])
    # Adding labels for the plot
    ax[1].set_axis_on()
    ax[1].tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    ax[1].set_xlabel('X (m)')
    ax[1].set_ylabel('Y (m)')
    ax[1].legend()
    ax[1].set_title('Optimal solution')
    
def plotting_data_from_excel():
    
    df = pd.read_excel(r'D:\TAMU\Research\Presentations\Discussions with Dr. Rathinam and Dr. Darbha\Spring 2023 files\Results Apr 10 MD algo improvement\Three_vehicle_20_targets\Data_three_vehicle_20_target.xlsx')
    
    # Plotting the deviations
    plt.figure()
    plt.plot(df['Instance no'], df['Deviation from optimum local search no swap'], 'r+', linewidth = 1.5, label = 'MD algo local search')
    plt.plot(df['Instance no'], df['Deviation from optimum no swap'], 'bo', linewidth = 1.5, label = 'MD algo perturbation', fillstyle = None)
    plt.plot(df['Instance no'], df['Deviation from optimum local search swap'], 'gx', linewidth = 1.5,\
             label = 'MD algo (swap) local search', fillstyle = None)
    plt.plot(df['Instance no'], df['Deviation from optimum swap'], 'kv', linewidth = 1.5, label = 'MD algo (swap) perturbation')
    plt.xlabel('Instance number')
    plt.ylabel('Percentage deviation of MD solution from optimum')
    plt.title('Percentage deviation for ' + str(3) + ' vhcls, '\
              + str(20) + ' targets')
    plt.legend()
    plt.show()
    
    # Plotting the runtimes
    plt.figure()
    plt.plot(df['Instance no'], df['Optimum runtime'], 'r+', linewidth = 1.5, label = 'Optimum')
    plt.plot(df['Instance no'], df['MD runtime no swap'], 'b*', linewidth = 1.5, label = 'MD algorithm')
    plt.plot(df['Instance no'], df['MD runtime'], 'gd', linewidth = 1.5, label = 'MD algorithm (swap)')
    plt.xlabel('Instance number')
    plt.ylabel('Runtime (s)')
    plt.title('Runtime of vehicles for ' + str(3) + ' vhcls, '\
              + str(20) + ' targets')
    plt.legend()
    plt.show()    
    
    # Plotting the runtimes for the two implementation of the MD algorithm
    plt.figure()
    plt.plot(df['Instance no'], df['MD runtime no swap'], 'b*', linewidth = 1.5, label = 'MD algorithm')
    plt.plot(df['Instance no'], df['MD runtime'], 'gd', linewidth = 1.5, label = 'MD algorithm (swap)')
    plt.xlabel('Instance number')
    plt.ylabel('Runtime (s)')
    plt.title('Runtime of vehicles for ' + str(3) + ' vhcls, '\
              + str(20) + ' targets')
    plt.legend()
    plt.show() 
    