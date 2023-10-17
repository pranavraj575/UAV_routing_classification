from route import *
from lkh import *
import random
import time
import numpy as np
import networkx as nx
from itertools import permutations

def targ_switch(route, param = ''): # param is not used. Created to be consistent with other neighbourhoods.
    # In this function, target switch neighbourhood is implemented, wherein
    # at each step, the vehicle with the maximum tour cost is considered and
    # a target is attempted to be removed from the vehicle

    # Keeping track of current solution in this neighbourhood
    curr_route = route.route_copy()

    # flag = 0 # flag to determine if all targets in the maximal vehicle's tour
    # # is considered, and no improvement is made
    # while flag == 0:

        # flag = 1 # Changing the flag variable, which will then be set to zero
        # only if an improvement is obtained

    # Obtaining vehicle with the maximum tour cost
    max_vhcl_no, max_cost = curr_route.vhcl_no_max_cost()
    # Obtaining sorted savings for targets in considered vehicle
    # start_time = time.time()
    savings_targ = curr_route.estimate_savings(max_vhcl_no)
    # print('Time taken to obtain savings is ', time.time() - start_time)
    # print("Savings for maximal target is ", savings_targ)
    
    # print('Vehicle tour costs in incumbent in VND ', curr_route.vhcl_tour_costs,' and tours are ', curr_route.vhcl_tours)
    # print('Current solution has cost ', curr_route.route_cost)

    # Running through each target in savings_targ and attempting to remove
    # targets with maximum potential savings from the tour
    for (t, _) in savings_targ:

        route_copy_target_switch = curr_route.route_copy()
        # Removing target from the vehicle with maximum tour cost
        # Checking if the vehicle is covering just one target
        if len(route_copy_target_switch.vhcl_tours[max_vhcl_no]) == 3:
            route_copy_target_switch.vhcl_tours[max_vhcl_no] = []
        else:
            route_copy_target_switch.vhcl_tours[max_vhcl_no].remove(t)
        
        # Inserting target to vehicle with minimum insertion cost
        # start_time = time.time()
        vhcl_no_min_insertion, _, pos_insertion = curr_route.vhcl_with_min_insertion_cost(t, max_vhcl_no)
        # print('Time taken to identify vehicle for insertion is ', time.time() - start_time)
        
        # Checking if the vehicle in which we are inserting the considered target
        # is visiting some other target. If pos_insertion == 0, then the vehicle is
        # not visiting any other target
        if pos_insertion == 0:
        
            route_copy_target_switch.vhcl_tours[vhcl_no_min_insertion] = [route_copy_target_switch._instance.vhcl_depot[vhcl_no_min_insertion],\
                                                                          t, route_copy_target_switch._instance.vhcl_depot[vhcl_no_min_insertion]]
        
        else:
        
            route_copy_target_switch.vhcl_tours[vhcl_no_min_insertion].insert(1, t)
        # Here, we insert at position 1 to ensure that we insert the target
        # somewhere in between the array. This is done to ensure that in the
        # array corresponding to the vehicle's tour, the first and last entry
        # is always the depot
        
        # Running LKH for these two vehicles and obtaining the tour
        # print('Vehicle tours after 1-opt are ', route_copy_target_switch.vhcl_tours, '. Vehicle with max cost is ', max_vhcl_no)
        # print('Maximal vehicle has targets ', route_copy_target_switch.vhcl_tours[max_vhcl_no][1:-1])
        # start_time = time.time()
        route_max_vhcl, tour_cost_max_vhcl =\
              construct_tour(route_copy_target_switch._instance, route_copy_target_switch.vhcl_tours[max_vhcl_no][1:-1],\
                             max_vhcl_no)
        route_ins_vhcl, tour_cost_ins_vhcl =\
                construct_tour(route_copy_target_switch._instance,\
                                route_copy_target_switch.vhcl_tours[vhcl_no_min_insertion][1:-1],\
                                vhcl_no_min_insertion)
        # end_time = time.time()
        # print('Time taken to construct LKH tours is ', end_time - start_time)
        
        # Modifying the routes
        # start_time = time.time()
        route_copy_target_switch.set_vhcl_i_tour_and_cost(route_max_vhcl, tour_cost_max_vhcl,\
                                                          max_vhcl_no)
        route_copy_target_switch.set_vhcl_i_tour_and_cost(route_ins_vhcl, tour_cost_ins_vhcl,\
                                                          vhcl_no_min_insertion)
        

        # Checking if the maximum tour cost has reduced
        # _, max_cost_after = route_copy_target_switch.vhcl_no_max_cost()
        # end_time = time.time()
        # print('Time taken to modify routes is ', end_time - start_time)
        
        if route_copy_target_switch.strictly_better_than(curr_route):

            # Changing to current solution
            # print('Better solution obtained. Changing to solution ', route_copy_target_switch.vhcl_tours)
            curr_route = route_copy_target_switch.route_copy()
            # flag = 0
            return curr_route            

    return curr_route

def one_opt(route, num_vhcls = 2, mode = 'i'): # IF MODE == 't', THEN TARGET SWITCH IS PERFORMED
    # WITH VEHICLES OF LEAST TOTAL TOUR COST AFTER INSERTION FOR "NUM_VHCLS" NUMBER OF VEHICLES.
    # IF MODE == 'i', THEN SWITCH IS PERFORMED WITH VEHICLES OF LEAST INSERTION COSTS.
    
    # In this function, one-opt switch is done using a metric with the top two
    # vehicles that have the least insertion cost for a considered target. 

    # Keeping track of current solution in this neighbourhood
    curr_route = route.route_copy()

    # Obtaining vehicle with the maximum tour cost
    max_vhcl_no, _ = curr_route.vhcl_no_max_cost()
    # Obtaining sorted savings for targets in considered vehicle
    savings_targ = curr_route.estimate_savings(max_vhcl_no)

    # print("Savings for maximal target is ", savings_targ)
    
    # print('Vehicle tour costs in incumbent in VND ', curr_route.vhcl_tour_costs,' and tours are ',\
    #  curr_route.vhcl_tours)
    # print('Current solution has cost ', curr_route.route_cost)

    # Running through each target in savings_targ and attempting to remove
    # targets with maximum potential savings from the tour
    for (t, saving) in savings_targ:    

        if mode == 'i':
            # Identifying the top "num_vhcls" number of vehicles with the least insertion cost
            vhcl_insertion_nos, insertion_cost_nos, pos_insertion_nos =\
                  route.sorted_vhcls_from_least_insertion_cost(t, max_vhcl_no, num_vhcls)
        elif mode == 't':
            # Identifying the top "num_vhcls" number of vehicles with the least tour costs
            vhcl_insertion_nos, total_cost_nos, pos_insertion_nos =\
                  route.sorted_vhcls_from_least_tot_tour_cost(t, max_vhcl_no, num_vhcls)
            # print(vhcl_insertion_nos, total_cost_nos, pos_insertion_nos)

        # Running through the possible vehicles for the target switch
        for i in range(len(vhcl_insertion_nos)):

            route_copy_target_switch = curr_route.route_copy()
            # Removing target from the vehicle with maximum tour cost
            # Checking if the vehicle is covering just one target
            if len(route_copy_target_switch.vhcl_tours[max_vhcl_no]) == 3:
                route_copy_target_switch.vhcl_tours[max_vhcl_no] = []
            else:
                route_copy_target_switch.vhcl_tours[max_vhcl_no].remove(t)

            # Inserting the target onto the considered vehicle
            if pos_insertion_nos[i] == 0: # In this case, the considered vehicle for insertion does
            # not visit any other target
                
                route_copy_target_switch.vhcl_tours[vhcl_insertion_nos[i]] = \
                    [route_copy_target_switch._instance.vhcl_depot[vhcl_insertion_nos[i]], t,\
                      route_copy_target_switch._instance.vhcl_depot[vhcl_insertion_nos[i]]]
            
            else:
            
                route_copy_target_switch.vhcl_tours[vhcl_insertion_nos[i]].insert(pos_insertion_nos[i], t)
                
            # Obtaining the estimated tour cost for considered vehicle for insertion
            if mode == 'i':            
                estimated_tour_cost_alt_vhcl = route_copy_target_switch.vhcl_tour_costs[vhcl_insertion_nos[i]]\
                      + insertion_cost_nos[i][1]
                      
            elif mode == 't':
                estimated_tour_cost_alt_vhcl = total_cost_nos[i][1]
            
            # print('Removing target ', t, ' from max vehicle and inserting it to ', vhcl_insertion_nos[i],\
            #       ' at position ', pos_insertion_nos[i])
            # print('Tour of insertion vehicle before and after is ', curr_route.vhcl_tours[vhcl_insertion_nos[i]],\
            #       route_copy_target_switch.vhcl_tours[vhcl_insertion_nos[i]])
            
            # Checking if the new estimated maximum tour cost is less than the incumbent
            route_copy_target_switch.set_vhcl_i_tour_cost(route_copy_target_switch.vhcl_tour_costs[max_vhcl_no]\
                                                          - saving, max_vhcl_no)
            route_copy_target_switch.set_vhcl_i_tour_cost(estimated_tour_cost_alt_vhcl, vhcl_insertion_nos[i])
            
            # Obtaining the new estimated max tour cost
            # _, max_cost_estimated = route_copy_target_switch.vhcl_no_max_cost()

            # Checking if obtained solution is better than the previous solution
            if route_copy_target_switch.strictly_better_than(curr_route):

                # Updating the solution
                # print('Better solution obtained.')
                curr_route = route_copy_target_switch.route_copy()
                # Improving the tours of the maximal vehicle and the current vehicle
                # using LKH
                max_vhcl_route, max_vhcl_cost = construct_tour(route._instance, curr_route.vhcl_tours[max_vhcl_no][1:-1],\
                                                                max_vhcl_no)
                curr_route.set_vhcl_i_tour_and_cost(max_vhcl_route, max_vhcl_cost, max_vhcl_no)
                vhcl_insert_route, vhcl_insert_cost =\
                      construct_tour(route._instance, curr_route.vhcl_tours[vhcl_insertion_nos[i]][1:-1],\
                                      vhcl_insertion_nos[i])
                curr_route.set_vhcl_i_tour_and_cost(vhcl_insert_route, vhcl_insert_cost, vhcl_insertion_nos[i])
                return curr_route
            
    return curr_route

def two_opt(route, num_vhcls = 2): # In this function, two-point swap is performed
    # with "num_vhcls" number of vehicles with least insertion cost using a metric.

    # print(num_vhcls)
    # Obtaining vehicle with the maximum tour cost
    max_vhcl_no, _ = route.vhcl_no_max_cost()
    # Obtaining sorted savings for targets in considered vehicle
    savings_targ = route.estimate_savings(max_vhcl_no)
    
    # print('Vehicle tours are ', route.vhcl_tours)
    # Running through each target in savings_targ and attempting to swap
    # targets
    for (t, saving) in savings_targ:

        # Obtaining sorted list of vehicles with which a swap will be attempted
        # to be performed. Note that the vehicles have been sorted in the increasing
        # order of insertion cost
        vhcl_nos_insertion, cost_insertion, pos_insertion =\
              route.sorted_vhcls_from_least_insertion_cost_for_swap(t, max_vhcl_no, num_vhcls)
        # print('Vehicles for insertion for ', t, 'is ', vhcl_nos_insertion, ' costs of insertion are', cost_insertion,\
        #         'and positions for insertion are ', pos_insertion)
    
        # Checking if there is at least on vehicle to swap with
        if not vhcl_nos_insertion:
            return route # Since there is no vehicle to swap with
        
        # Running through all possible vehicles
        for i in range(len(vhcl_nos_insertion)):

            # Making a copy of the current solution
            route_copy_t_swap = route.route_copy()
            # Removing target from the vehicle with maximum tour cost
            # Checking if the vehicle is covering just one target
            if len(route_copy_t_swap.vhcl_tours[max_vhcl_no]) == 3:
                route_copy_t_swap.vhcl_tours[max_vhcl_no] = []
            else:
                route_copy_t_swap.vhcl_tours[max_vhcl_no].remove(t)
            
            # Inserting the considered target in the chosen vehicle's tour.
            # NOTE HERE THAT THE VEHICLE WITH WHICH WE ARE SWAPPING WILL HAVE AT LEAST ONE TARGET
            # BEFOREHAND. HENCE, POS_INSERTION WILL ALWAYS BE NON-ZERO.
            route_copy_t_swap.vhcl_tours[vhcl_nos_insertion[i]].insert(pos_insertion[i], t)

            # Updating the estimated tour costs of the vehicles
            route_copy_t_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]\
                                                    - saving, max_vhcl_no)
            route_copy_t_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[vhcl_nos_insertion[i]]\
                                                   + cost_insertion[i][1], vhcl_nos_insertion[i])
            
            # # Performing a 2-opt swap for both the vehicles
            # route_copy_t_swap.two_opt_edges(max_vhcl_no)
            # route_copy_t_swap.two_opt_edges(vhcl_nos_insertion[i])
            # # Obtaining new "saving" obtained for the maximal vehicle, which would
            # # potentially be more than the initial estimated saving after a two-swap
            # saving = route.vhcl_tour_costs[max_vhcl_no] - route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]

            #---------------Rule for going through targets in other vehicle
            # Obtaining sorted list of targets in considered vehicle; sorted based
            # on increasing order of "increase cost" metric for the maximal vehicle.
            # The goal is to swap a target from the maximal vehicle with high savings
            # with a target that has a low increase in cost.
            increase_cost_t_swap = route_copy_t_swap.vhcl_i_insertion_sort_targets(route.vhcl_tours[vhcl_nos_insertion[i]][1:-1],\
                                                                                    max_vhcl_no, vhcl_nos_insertion[i])
            # print('Sorted targets in other vehicle based on increasing insertion cost is ', increase_cost_t_swap)
            
            # Running through targets in the list
            for ((p, pos), inc) in increase_cost_t_swap:

                # Checking if increase in cost associated with inserting p in the
                # maximal vehicle is more than the saving obtained by removing target
                # t. In this case, the loop is broken.
                if inc > saving:
                    # print('Estimated increase is ', inc, ' and estimated saving is ', saving, '.')
                    break

                # Obtaining the saving associated with removing p from the
                # vehicle in which target "t" was inserted
                saving_p = route_copy_t_swap.estimate_savings_remove_targ(vhcl_nos_insertion[i], p)

                # Updating the vehicle routes
                route_copy_t_swap_with_p = route_copy_t_swap.route_copy()

                # Removing p from vhcl_insertion and inserting it to maximal vehicle.
                route_copy_t_swap_with_p.vhcl_tours[vhcl_nos_insertion[i]].remove(p)
                # Checking if the maximal vehicle has an empty tour or not
                if len(route_copy_t_swap_with_p.vhcl_tours[max_vhcl_no]) < 3:
                    route_copy_t_swap_with_p.vhcl_tours[max_vhcl_no] = \
                        [route_copy_t_swap_with_p._instance.vhcl_depot[max_vhcl_no], p,\
                        route_copy_t_swap_with_p._instance.vhcl_depot[max_vhcl_no]]
                else:
                    route_copy_t_swap_with_p.vhcl_tours[max_vhcl_no].insert(pos, p)

                # Updating the tour costs for the two vehicles
                route_copy_t_swap_with_p.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[vhcl_nos_insertion[i]]\
                                                               - saving_p, vhcl_nos_insertion[i])
                route_copy_t_swap_with_p.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[max_vhcl_no] + inc,\
                                                                max_vhcl_no)
                # Performing a two-opt edge swap for the two vehicles
                route_copy_t_swap_with_p.two_opt_edges(max_vhcl_no)
                route_copy_t_swap_with_p.two_opt_edges(vhcl_nos_insertion[i])

                # Checking if the obtained solution is better than the current solution
                if route_copy_t_swap_with_p.strictly_better_than(route):
                    
                    # The tours of the maximal vehicle and the other vehicle are
                    # improved by using LKH
                    # print('Better solution obtained.')
                    route = route_copy_t_swap_with_p.route_copy()
                    max_vhcl_route, max_vhcl_cost = construct_tour(route._instance, route.vhcl_tours[max_vhcl_no][1:-1],\
                                                                    max_vhcl_no)
                    route.set_vhcl_i_tour_and_cost(max_vhcl_route, max_vhcl_cost, max_vhcl_no)
                    vhcl_insertion_route, vhcl_insertion_cost =\
                        construct_tour(route._instance, route.vhcl_tours[vhcl_nos_insertion[i]][1:-1],\
                                        vhcl_nos_insertion[i])
                    route.set_vhcl_i_tour_and_cost(vhcl_insertion_route, vhcl_insertion_cost, vhcl_nos_insertion[i])
                    return route 
            
    return route

def one_opt_neighbours(route, num_targ_neighbours = 30):
    # In this function, a one-opt swap of targets is attempted to be made with
    # targets in the vicinity. Here, num_targ_neighbours controls the number of
    # targets that are to be explored in the vicinity.

    # Obtaining vehicle with the maximum tour cost
    max_vhcl_no, _ = route.vhcl_no_max_cost()
    
    # Obtaining the list of targets that are not in the maximal vehicle's tour
    # and that have not be pre-assigned to another vehicle.
    targets_not_in_tour_with_vhcl = route.obtain_targets_not_in_tour(max_vhcl_no)

    # Obtaining the savings for targets in the maximal vehicle
    savings_targ = route.estimate_savings(max_vhcl_no)

    # Running through each target in savings_targ and attempting to remove
    # targets with maximum potential savings from the tour
    for (t, saving) in savings_targ:

        # Obtaining list of targets in the vicinity of the considered target
        # Making a copy of the route and removing t
        route_copy_remove_t = route.route_copy()
        # Checking if the vehicle is covering just one target
        if len(route_copy_remove_t.vhcl_tours[max_vhcl_no]) == 3:
            route_copy_remove_t.vhcl_tours[max_vhcl_no] = []
        else:
            route_copy_remove_t.vhcl_tours[max_vhcl_no].remove(t)
        # Updating the tour cost for the maximal vehicle
        route_copy_remove_t.set_vhcl_i_tour_cost(route_copy_remove_t.vhcl_tour_costs[max_vhcl_no]\
                                                  - saving, max_vhcl_no)
        
        # Obtaining the list of targets in its vicinity
        targ_vicinity = route.get_targets_vicinity(t, targets_not_in_tour_with_vhcl, num_targ_neighbours)

        # Running through the targets in the vicinity
        for ((t_other, vhcl_ind, pos), _) in targ_vicinity:

            # In this case, target t is removed from the maximal vehicle and inserted
            # after t_other. Once this is done, a two_opt search for edges is performed
            # in the current vehicle
            route_switch = route_copy_remove_t.route_copy()

            # Inserting t after target t_other. Note that "pos" is the index in which
            # t_other is located in vhcl_ind's tour.
            # Obtaining the insertion cost at location pos + 1
            t_front = route_switch.vhcl_tours[vhcl_ind][pos + 1]
            insertion_cost = route_switch._instance.get_vhcl_cost(t_other, t, vhcl_ind) \
                + route_switch._instance.get_vhcl_cost(t, t_front, vhcl_ind) \
                - route_switch._instance.get_vhcl_cost(t_other, t_front, vhcl_ind)
            # Inserting target t
            route_switch.vhcl_tours[vhcl_ind].insert(pos + 1, t)
            # Updating the tour cost
            route_switch.set_vhcl_i_tour_cost(route_switch.vhcl_tour_costs[vhcl_ind] + insertion_cost,\
                                              vhcl_ind)
            # Performing a two-opt swap of edges for the considered vehicle
            route_switch.two_opt_edges(vhcl_ind)

            # Checking if a better solution has been obtained
            if route_switch.strictly_better_than(route):

                # Updating the best solution, and running LKH for the maximal vehicle
                # and other vehicle
                route = route_switch.route_copy()
                # Running LKH on the maximal vehicle
                max_vhcl_route, max_vhcl_cost = construct_tour(route._instance, route.vhcl_tours[max_vhcl_no][1:-1],\
                                                                max_vhcl_no)
                route.set_vhcl_i_tour_and_cost(max_vhcl_route, max_vhcl_cost, max_vhcl_no)
                # Running LKH on the vehicle in which the target was inserted
                vhcl_insert_route, vhcl_insert_cost =\
                        construct_tour(route._instance, route.vhcl_tours[vhcl_ind][1:-1],\
                                        vhcl_ind)
                route.set_vhcl_i_tour_and_cost(vhcl_insert_route, vhcl_insert_cost, vhcl_ind)
                return route
            
    return route

# def two_opt_neighbours(route, num_targ_neighbours = 30):
#     # In this function, a two-opt swap of targets is attempted to be made with
#     # targets in the vicinity. Here, num_targ_neighbours controls the number of
#     # targets that are to be explored in the vicinity.

#     # Obtaining vehicle with the maximum tour cost
#     max_vhcl_no, _ = route.vhcl_no_max_cost()
    
#     # Obtaining the list of targets that are not in the maximal vehicle's tour
#     # and that have not been pre-assigned to another vehicle.
#     targets_not_in_tour_with_vhcl = route.obtain_targets_not_in_tour(max_vhcl_no)

#     # Obtaining the savings for targets in the maximal vehicle
#     savings_targ = route.estimate_savings(max_vhcl_no)

#     # Running through each target in savings_targ and attempting to remove
#     # targets with maximum potential savings from the tour
#     for (t, saving) in savings_targ:

#         # Obtaining list of targets in the vicinity of the considered target
#         # Making a copy of the route and removing t
#         route_copy_remove_t = route.route_copy()
#         # Obtaining the position in which t is in the maximal vehicle
#         pos_t = route_copy_remove_t.vhcl_tours[max_vhcl_no].index(t)
#         # Removing target t from the maximal vehicle. NOTE THAT EVEN IF THE MAXIMAL
#         # VEHICLE HAS JUST ONE TARGET, WE REMOVE DIRECTELY, SINCE LATER, A TARGET
#         # T_OTHER WILL BE INSERTED.
#         route_copy_remove_t.vhcl_tours[max_vhcl_no].remove(t)
#         # Updating the tour cost for the maximal vehicle
#         route_copy_remove_t.set_vhcl_i_tour_cost(route_copy_remove_t.vhcl_tour_costs[max_vhcl_no]\
#                                                   - saving, max_vhcl_no)
        
#         # Obtaining the list of targets in its vicinity
#         targ_vicinity = route.get_targets_vicinity(t, targets_not_in_tour_with_vhcl, num_targ_neighbours)

#         for ((t_other, vhcl_ind, pos), _) in targ_vicinity:

#             # Making a copy of the route object
#             route_copy_swap_t_t_other = route_copy_remove_t.route_copy()

#             # Obtaining the savings associated with removing t_other
#             savings_t_other = route.estimate_savings_remove_targ(vhcl_ind, t_other)

#             # Removing t_other from the considered vehicle, and updating the tour cost
#             route_copy_remove_t.vhcl_tours[vhcl_ind].remove(t_other)
#             route_copy_remove_t.set_vhcl_i_tour_cost(route_copy_remove_t.vhcl_tour_costs[vhcl_ind]\
#                                                      - savings_t_other, vhcl_ind)

#             # Obtaining the insertion cost associated with t_other in the maximal
#             # vehicle, and t in the other vehicle
#             # insertion_cost_t_other = 

def mult_targ_swap(route, num_targ = 2, num_candidates_swap = 20):
    # In this function, num_targ number of targets from the maximal vehicle are swapped with
    # num_targ number of targets or num_targ - 1 number of targets from another vehicle using a metric.

    # print(num_targ)
    # Obtaining vehicle with the maximum tour cost
    max_vhcl_no, _ = route.vhcl_no_max_cost()

    # Obtaining the savings of all pairs of targets
    # print('Tour of maximal vehicle, and targets pre-allocated are ', route.vhcl_tours[max_vhcl_no],\
    #       route._instance.targ_alloc_to_each_vhcl[max_vhcl_no])
    savings_mult_targ = route.savings_vehicle_mult_targ_sorted(max_vhcl_no, num_targ)
    # print(savings_mult_targ)

    # Running through each set of targets that could be removed
    for (targ_arr, saving) in savings_mult_targ:

        # print('Attempting to replace', targ_arr)
        route_copy_t_swap = route.route_copy()
        # Removing the considered target array from the maximal vehicle
        if len(route_copy_t_swap.vhcl_tours[max_vhcl_no]) == num_targ + 2:
            route_copy_t_swap.vhcl_tours[max_vhcl_no] = []
        else:
            for t in targ_arr:
                route_copy_t_swap.vhcl_tours[max_vhcl_no].remove(t)

        #----------- Rule for choosing vehicle to swap with
        # Picking vehicle with minimum insertion cost to swap with. Note that the
        # vehicles considered for insertion are such that there are atleast
        # "num_targ" number of common targets available to swap with.
        vhcl_insertion, insertion_cost, pos_insertion, orientation =\
              route.vhcl_with_min_insertion_cost_for_swap_mult_targ(targ_arr, max_vhcl_no)
        # print('Vehicle for insertion, insertion cost, and position and orientation are ',\
        #       vhcl_insertion, insertion_cost, pos_insertion, orientation)
        
        # Checking if there is at least one vehicle to swap with
        if np.isnan(vhcl_insertion):
            return route
        
        # Obtaining estimated tour cost for vehicle in which t is attempted to be inserted
        best_tour_cost_vhcl_except_maximal = insertion_cost + route.vhcl_tour_costs[vhcl_insertion]

        # In this case, the considered target array is inserted in the tour of the other vehicle.
        # NOTE HERE THAT THE VEHICLE WITH WHICH WE ARE SWAPPING WILL HAVE AT LEAST SOME TARGETS BEFOREHAND.
        # HENCE, POS_INSERTION WILL ALWAYS BE NON-ZERO.

        # Inserting the target array in "pos_insertion"
        # print('Tour of insertion vehicle is ', route_copy_t_swap.vhcl_tours[vhcl_insertion])
        # Checking orientation for the insertion
        if orientation == 0:
            route_copy_t_swap.vhcl_tours[vhcl_insertion][pos_insertion:pos_insertion] = list(targ_arr)
        else:
            targ_arr_switched = [targ_arr[i] for i in reversed(range(len(targ_arr)))]
            route_copy_t_swap.vhcl_tours[vhcl_insertion][pos_insertion:pos_insertion] = targ_arr_switched
        # print('Tour of insertion vehicle after inserting ', targ_arr, 'at position', pos_insertion,\
        #       'with orientation ', orientation, 'is ', route_copy_t_swap.vhcl_tours[vhcl_insertion])

        # Updating the estimated tour costs of the vehicles
        route_copy_t_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]\
                                                - saving, max_vhcl_no)
        route_copy_t_swap.set_vhcl_i_tour_cost(best_tour_cost_vhcl_except_maximal, vhcl_insertion)
        # Performing a 2-opt swap of edges for the two vehicles
        route_copy_t_swap.two_opt_edges(max_vhcl_no)
        route_copy_t_swap.two_opt_edges(vhcl_insertion)

        # # Obtaining the new "savings" for the maximal vehicle
        # saving_mod = route.vhcl_tour_costs[max_vhcl_no] - route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]

        #------------------Rule for going through target sets in other vehicle
        # Obtaining sorted list of target sets in considered vehicle; sorted based
        # on increasing order of "increase cost" metric for the maximal vehicle.
        # The goal is to swap a target set from the maximal vehicle with high savings
        # with a target set that has a low increase in cost.
        increase_cost_targ_arr_swap =\
              route_copy_t_swap.vhcl_i_mult_insertion_sort_targets(targ_arr, num_targ, max_vhcl_no, vhcl_insertion)

        # Going through the top num_candidates_swap number of chunks for the swap
        ind = 0
        for ((targ_arr_swap, pos_ins, orientation_ins), increase) in increase_cost_targ_arr_swap:
            
            # Obtaining the saving associated with removing targets in targ_arr_swap from
            # the vehicle in which target array "targ_arr" was inserted
            saving_targ_arr_swap = route_copy_t_swap.savings_vehicle_removing_mult_targ(vhcl_insertion, targ_arr_swap)

            # Updating the vehicle routes
            route_copy_t_swap_with_targ_arr_swap = route_copy_t_swap.route_copy()

            # Removing targ_arr_swap from vhcl_insertion and inserting it to maximal
            # vehicle.
            for t in targ_arr_swap:
                route_copy_t_swap_with_targ_arr_swap.vhcl_tours[vhcl_insertion].remove(t)

            # Checking the orientation for insertion
            if orientation_ins == 0:
                targ_arr_insert = list(targ_arr_swap)
            else:
                targ_arr_insert = [targ_arr_swap[i] for i in reversed(range(len(targ_arr_swap)))]
            
            # print('Maximal vehicle tour before insertion is ', route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no])
            # Checking if the maximal vehicle has an empty tour or not
            if len(route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no]) < 3:
                route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no] = targ_arr_insert
                # Inserting the depot at the beginning and at the end
                route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no].insert(0,\
                                        route_copy_t_swap_with_targ_arr_swap._instance.vhcl_depot[max_vhcl_no])
                route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no].insert(-1,\
                                        route_copy_t_swap_with_targ_arr_swap._instance.vhcl_depot[max_vhcl_no])
            else:
                 route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no][pos_ins:pos_ins] = targ_arr_insert

            # print('Maximal vehicle tour after insertion of ', targ_arr_swap, pos_ins, orientation_ins, ' is ',\
            #       route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no])

            # Updating the tour costs for the two vehicles
            route_copy_t_swap_with_targ_arr_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[vhcl_insertion]\
                                                                      -saving_targ_arr_swap, vhcl_insertion)
            route_copy_t_swap_with_targ_arr_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]\
                                                                      + increase, max_vhcl_no)
            # Performing a 2-opt swap of edges for the two vehicles
            route_copy_t_swap_with_targ_arr_swap.two_opt_edges(vhcl_insertion)
            route_copy_t_swap_with_targ_arr_swap.two_opt_edges(max_vhcl_no)
            
            # print('Maximal vehicle tour after 2-opt swap is ', route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no])

            # Checking if a better solution is obtained
            if route_copy_t_swap_with_targ_arr_swap.strictly_better_than(route):

                # The tours of the maximal vehicle and the other vehicle are improved
                # using LKH
                route = route_copy_t_swap_with_targ_arr_swap.route_copy()
                max_vhcl_route, max_vhcl_cost = construct_tour(route._instance, route.vhcl_tours[max_vhcl_no][1:-1],\
                                                               max_vhcl_no)
                route.set_vhcl_i_tour_and_cost(max_vhcl_route, max_vhcl_cost, max_vhcl_no)
                vhcl_insertion_route, vhcl_insertion_cost =\
                      construct_tour(route._instance, route.vhcl_tours[vhcl_insertion][1:-1],\
                                     vhcl_insertion)
                route.set_vhcl_i_tour_and_cost(vhcl_insertion_route, vhcl_insertion_cost, vhcl_insertion)
                return route

            # Updating the index for number of candidate swaps explored
            ind += 1
            if ind == num_candidates_swap:
                break
    
    return route

def mult_targ_swap_dynamic(route, max_num_targ_chunk = 7, num_candidates_swap = 200):
    # In this function, target chunks from the maximal vehicle with at least
    # two targets and at most "max_num_targ" number of targets are swapped with at least
    # "max_num_targ" number of targets from the vehicle with the least insertion cost

    # print('Maximum chunk size is ', max_num_targ_chunk)

    # Keeping track of runtime in the neighborhood
    read_write_time = 0

    # Obtaining the maximal vehicle
    max_vhcl_no, _ = route.vhcl_no_max_cost()

    # Obtaining the sorted list of targets from the maximal vehicle
    removal_ratio_mult_targs = route.removal_ratio_targets_mult_targ(max_vhcl_no, max_num_targ_chunk)
    # print(removal_ratio_mult_targs)

    # Running through each chunk of targets in the sorted list
    for (targ_arr, _) in removal_ratio_mult_targs:
        
        # print('Attempting to remove ', targ_arr, ' from the maximal vehicle.')

        route_copy_t_swap = route.route_copy()
        # Obtaining the saving corresponding to the removed target array from the 
        # maximal vehicle
        saving = route_copy_t_swap.savings_vehicle_removing_mult_targ(max_vhcl_no, targ_arr)
        # print(saving)
        # Removing the considered target array from the maximal vehicle
        for t in targ_arr:
            route_copy_t_swap.vhcl_tours[max_vhcl_no].remove(t)
        # Checking if the remaining tour contains only the depots. If so,
        # the tour of the maximal vehicle is set to an empty array
        if len(route_copy_t_swap.vhcl_tours[max_vhcl_no]) == 2:
            route_copy_t_swap.vhcl_tours[max_vhcl_no] = []
            
        # print('Maximal tour after removing ', targ_arr, ' is ', route_copy_t_swap.vhcl_tours[max_vhcl_no])

        #----------- Rule for choosing vehicle to swap with
        # Picking vehicle with minimum insertion cost. Note that the vehicle considered
        # is such that there is atleast one common target to perform a swap with
        vhcl_insertion, insertion_cost, pos_arr_insertion =\
              route.vhcl_with_min_insertion_cost_mult_targ_recursive(targ_arr, max_vhcl_no)
        # print(vhcl_insertion)
        # print(insertion_cost)
        
        # Checking if there is at least one vehicle to swap with
        if np.isnan(vhcl_insertion):
            return route
        
        # Updating the tour of the vehicle in which targ_arr must be inserted, and
        # the tour costs for the considered vehicle
        for i in range(len(targ_arr)):
            route_copy_t_swap.vhcl_tours[vhcl_insertion].insert(pos_arr_insertion[i], targ_arr[i])

        # print('Other vehicles tour after insertion is ', route_copy_t_swap.vhcl_tours[vhcl_insertion])

        # Updating the estimated tour costs of the vehicles
        route_copy_t_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]\
                                               - saving, max_vhcl_no)
        route_copy_t_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[vhcl_insertion]\
                                               + insertion_cost, vhcl_insertion)
        
        #-----------Rule for going through target sets in the other vehicle
        savings_insertion_ratio_sorted =\
              route_copy_t_swap.vhcl_i_savings_insertion_ratio_mult_sort_targets(targ_arr, max_num_targ_chunk,\
                                                                                 max_vhcl_no, vhcl_insertion)
        # raise Exception('Test')
        # print('Targets from other vehicle sorted using the savings/insertion cost ratio is ',\
        #        savings_insertion_ratio_sorted)

        # Going through the top num_candidates_swap number of chunks for the swap
        ind = 0
        for ((targ_arr_swap, pos_arr_insertion_other, insert, saving_other), _) in savings_insertion_ratio_sorted:

            # print('Attempting swap with ', targ_arr_swap)
            # Updating the vehicle routes
            route_copy_t_swap_with_targ_arr_swap = route_copy_t_swap.route_copy()

            # Removing targ_arr_swap from vhcl_insertion
            for t in targ_arr_swap:
                route_copy_t_swap_with_targ_arr_swap.vhcl_tours[vhcl_insertion].remove(t)

            # Inserting targets in the maximal vehicle
            for i in range(len(targ_arr_swap)):
                # Checking if pos_arr_insertion_other[0] = 0, i.e., if the maximal
                # vehicle's tour is empty to begin with
                if i == 0 and len(route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no]) <= 2:
                    route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no] =\
                          [route._instance.vhcl_depot[max_vhcl_no], targ_arr_swap[0],\
                            route._instance.vhcl_depot[max_vhcl_no]]
                else:
                    route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no].insert(pos_arr_insertion_other[i],\
                                                                                        targ_arr_swap[i])
            
            # Updating the tour costs
            route_copy_t_swap_with_targ_arr_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[vhcl_insertion]\
                                                                      -saving_other, vhcl_insertion)
            route_copy_t_swap_with_targ_arr_swap.set_vhcl_i_tour_cost(route_copy_t_swap.vhcl_tour_costs[max_vhcl_no]\
                                                                      + insert, max_vhcl_no)
            # Performing a 2-opt swap of edges for the two vehicles
            route_copy_t_swap_with_targ_arr_swap.two_opt_edges(vhcl_insertion)
            route_copy_t_swap_with_targ_arr_swap.two_opt_edges(max_vhcl_no)

            # Running LKH on the two vehicles
            # print('Running LKH')
            # start_time = time.time()
            max_vhcl_route, max_vhcl_cost,\
                read_write_time_inc = construct_tour(route_copy_t_swap_with_targ_arr_swap._instance,\
                                                            route_copy_t_swap_with_targ_arr_swap.vhcl_tours[max_vhcl_no][1:-1],\
                                                            max_vhcl_no)
            # end_time = time.time()
            # print(end_time - start_time)
            route_copy_t_swap_with_targ_arr_swap.set_vhcl_i_tour_and_cost(max_vhcl_route, max_vhcl_cost, max_vhcl_no)
            read_write_time += read_write_time_inc
            
            vhcl_insertion_route, vhcl_insertion_cost, read_write_time_inc =\
                      construct_tour(route_copy_t_swap_with_targ_arr_swap._instance,\
                                      route_copy_t_swap_with_targ_arr_swap.vhcl_tours[vhcl_insertion][1:-1],\
                                      vhcl_insertion)
            route_copy_t_swap_with_targ_arr_swap.set_vhcl_i_tour_and_cost(vhcl_insertion_route,\
                                                                        vhcl_insertion_cost, vhcl_insertion)
            read_write_time += read_write_time_inc
            
            # print('Vehicle tours before swapping are ', route.vhcl_tours, ' and after swapping ',\
            #       targ_arr, ' and ', targ_arr_swap, ' are ', route_copy_t_swap_with_targ_arr_swap.vhcl_tours)
   
            # print(read_write_time)

            # Checking if a better solution is obtained
            if route_copy_t_swap_with_targ_arr_swap.strictly_better_than(route):

                # The tours of the maximal vehicle and the other vehicle are improved
                # using LKH
                print('Better solution is obtained. Target chunk removed from other vehicle is ', targ_arr_swap,\
                      '. Target chunk removed from maximal vehicle is', targ_arr)
                # route = route_copy_t_swap_with_targ_arr_swap.route_copy()
                # max_vhcl_route, max_vhcl_cost, _ = construct_tour(route._instance, route.vhcl_tours[max_vhcl_no][1:-1],\
                #                                                 max_vhcl_no)
                # route.set_vhcl_i_tour_and_cost(max_vhcl_route, max_vhcl_cost, max_vhcl_no)
                # vhcl_insertion_route, vhcl_insertion_cost, _ =\
                #       construct_tour(route._instance, route.vhcl_tours[vhcl_insertion][1:-1],\
                #                       vhcl_insertion)
                # route.set_vhcl_i_tour_and_cost(vhcl_insertion_route, vhcl_insertion_cost, vhcl_insertion)
                print('Vehicle tours are ', route.vhcl_tours)
                return route_copy_t_swap_with_targ_arr_swap, read_write_time

            # Updating the index for number of candidate swaps explored
            ind += 1
            if ind == num_candidates_swap:
                break
    
    return route, read_write_time


def shake_two_targ_swap(route):
    # In this function, a random alternate solution is generated by picking a
    # random target from the maximal vehicle (that is not to be necessarily covered
    # by that vehicle) and picking another random vehicle (that has atleast one common
    # target that it covers), choosing a random common target from its tour, and
    # swapping with the target picked from the maximal vehicle.

    # Obtaining a random target from the maximal vehicle's tour
    max_vhcl_no, _ = route.vhcl_no_max_cost()
    random_targ_max_vhcl = route.choosing_random_target(max_vhcl_no)
    # Picking a random vehicle from other vehicles that has a common target to swap with
    array_vhcls = route.vhcl_nos_with_replacable_targets_except_max_vhcl(max_vhcl_no)
    rand_vhcl = array_vhcls[random.randint(0, len(array_vhcls) - 1)]
    # Picking a random target from this vehicle
    random_targ_random_vhcl = route.choosing_random_target(rand_vhcl)

    # Making a copy of the route
    route_swap = route.route_copy()
    # Swapping the targets
    route_swap.vhcl_tours[max_vhcl_no].remove(random_targ_max_vhcl)
    route_swap.vhcl_tours[max_vhcl_no].insert(1, random_targ_random_vhcl)
    route_swap.vhcl_tours[rand_vhcl].remove(random_targ_random_vhcl)
    route_swap.vhcl_tours[rand_vhcl].insert(1, random_targ_max_vhcl)
    # Recomputing the tours for the two vehicles
    route_max_vhcl, tour_cost_max_vhcl = construct_tour(route._instance,\
                                                        route_swap.vhcl_tours[max_vhcl_no][1:-1],\
                                                        max_vhcl_no)
    route_rand_vhcl, tour_cost_rand_vhcl = construct_tour(route._instance,\
                                                          route_swap.vhcl_tours[rand_vhcl][1:-1],\
                                                            rand_vhcl)
    # Storing the obtained tours
    route_swap.set_vhcl_i_tour_and_cost(route_max_vhcl, tour_cost_max_vhcl, max_vhcl_no)
    route_swap.set_vhcl_i_tour_and_cost(route_rand_vhcl, tour_cost_rand_vhcl, rand_vhcl)

    return route_swap

def shake_two_targ_swap_top_five_targ(route):
    """In this function, shaking is performed corresponding to the top five targets
    in the maximal tour

    Args:
        route (_type_): _description_
    """

    # Obtaining the savings in the maximal vehicle
    max_vhcl_no = route.vhcl_no_max_cost()[0]
    savings = route.estimate_savings(max_vhcl_no)
    # Picking one of the top five targets corresponding to the savings
    random_targ_max_vhcl = np.random.choice([i[0] for i in savings[0:5]])
    # Picking a random vehicle from other vehicles that has a common target to swap with
    array_vhcls = route.vhcl_nos_with_replacable_targets_except_max_vhcl(max_vhcl_no)
    rand_vhcl = array_vhcls[random.randint(0, len(array_vhcls) - 1)]
    # Picking a random target from this vehicle
    random_targ_random_vhcl = route.choosing_random_target(rand_vhcl)

    # Making a copy of the route
    route_swap = route.route_copy()
    # Swapping the targets
    route_swap.vhcl_tours[max_vhcl_no].remove(random_targ_max_vhcl)
    route_swap.vhcl_tours[max_vhcl_no].insert(1, random_targ_random_vhcl)
    route_swap.vhcl_tours[rand_vhcl].remove(random_targ_random_vhcl)
    route_swap.vhcl_tours[rand_vhcl].insert(1, random_targ_max_vhcl)
    # Recomputing the tours for the two vehicles
    route_max_vhcl, tour_cost_max_vhcl = construct_tour(route._instance,\
                                                        route_swap.vhcl_tours[max_vhcl_no][1:-1],\
                                                        max_vhcl_no)
    route_rand_vhcl, tour_cost_rand_vhcl = construct_tour(route._instance,\
                                                          route_swap.vhcl_tours[rand_vhcl][1:-1],\
                                                            rand_vhcl)
    # Storing the obtained tours
    route_swap.set_vhcl_i_tour_and_cost(route_max_vhcl, tour_cost_max_vhcl, max_vhcl_no)
    route_swap.set_vhcl_i_tour_and_cost(route_rand_vhcl, tour_cost_rand_vhcl, rand_vhcl)

    return route_swap