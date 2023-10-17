import random
import numpy as np
import copy
from itertools import combinations

def percent_change(val1, val2):
    return (abs(val2 - val1) * 100.0) / max(val1, val2)

class Route:
# In this class, information about each vehicle's tour and tour cost is kept
# track of
    def __init__(self, instance):
        self.log = instance.log
        self._instance = instance
        self.vhcl_tour_costs = {}
        self.vhcl_tours = {}
        self.route_cost = 0
        self.targ_all_to_vhcls = {}

    def route_copy(self):
        other = Route(self._instance)
        other.vhcl_tour_costs = copy.deepcopy(self.vhcl_tour_costs)
        other.vhcl_tours = copy.deepcopy(self.vhcl_tours)
        other.route_cost = copy.deepcopy(self.route_cost)
        other.targ_all_to_vhcls = {}
        return other
    
    def get_targ_all_to_vhcls(self):
        # In this function, a dictionary is constructed such that for each target,
        # the vehicle number to which the target is allocated is obtained
        
        # Running through vehicle tours
        for i in range(len(self._instance.depots)):
            for j in range(1, len(self.vhcl_tours[i + 1]) - 1):
                self.targ_all_to_vhcls[self.vhcl_tours[i + 1][j]] = i + 1
    
    def set_vhcl_i_tour_cost(self, cost, vhcl_no):
        # Checking if vhcl_no = 1, 2, .., such that the maximum value is equal
        # to the number of depots
        assert(vhcl_no <= self._instance.get_num_vhcls())
        # Setting the vehicle with number vhcl_no's route to be the given route
        # and the vehicle's tour cost to be the given cost
        self.vhcl_tour_costs[vhcl_no] = cost
        
        # Updating the route cost as well
        max_route_cost = 0
        for i in range(self._instance.get_num_vhcls()):
            if (i + 1) in self.vhcl_tour_costs: # Checking if route for vehicle number
            # i + 1 exists
                max_route_cost = max(max_route_cost, self.vhcl_tour_costs[i + 1])
        
        self.route_cost = max_route_cost

    def set_vhcl_i_tour_and_cost(self, vhcl_route, cost, vhcl_no):
        # Checking if vhcl_no = 1, 2, .., such that the maximum value is equal
        # to the number of depots
        assert(vhcl_no <= self._instance.get_num_vhcls())
        # Setting the vehicle with number vhcl_no's route to be the given route
        # and the vehicle's tour cost to be the given cost
        self.vhcl_tours[vhcl_no] = vhcl_route
        self.vhcl_tour_costs[vhcl_no] = cost

        # Updating the route cost as well
        max_route_cost = 0
        for i in range(self._instance.get_num_vhcls()):
            if (i + 1) in self.vhcl_tour_costs: # Checking if route for vehicle number
            # i + 1 exists
                max_route_cost = max(max_route_cost, self.vhcl_tour_costs[i + 1])
        
        self.route_cost = max_route_cost

    def set_all_vhcls_tour_cost(self, routes, costs):

        # Checking that the route array and costs array have the same length as
        # the number of vehicles
        assert(len(routes) == self._instance.get_num_vhcls())
        assert(len(costs) == self._instance.get_num_vhcls())
        # Running through all vehicles and assigning routes and costs
        for i in range(self._instance.get_num_vhcls()):

            self.set_vhcl_i_tour_and_cost(routes[i], costs[i], i + 1)

    def vhcl_no_max_cost(self):

        # Returns the vehicle number with the maximum tour cost. Note that the
        # vehicles numbers are 1, 2, ...
        vhcl_no_max_cost = max(self.vhcl_tour_costs, key = self.vhcl_tour_costs.get)
        return vhcl_no_max_cost, self.vhcl_tour_costs[vhcl_no_max_cost]
    
    def vhcl_nos_with_replacable_targets_except_max_vhcl(self, vhcl_no_with_max_cost):

        # In this function, vehicle numbers that have replacable targets are
        # returned. 
        vhcl_nos = []
        for i in self.vhcl_tour_costs.keys():
            # Checking if the considered vehicle has a zero cost tour, i.e., it
            # does not cover any targets, or the considered vehicle is the vehicle
            # with the maximum tour cost
            if self.vhcl_tour_costs[i] == 0 or i == vhcl_no_with_max_cost:
                continue
            # Checking if in the tour of the considered vehicle, there is a target
            # that need not necessarily be covered by the considered vehicle
            for j in range(1, len(self.vhcl_tours[i]) - 1):
                if self.vhcl_tours[i][j] not in self._instance.targ_alloc_to_each_vhcl[i]:
                    vhcl_nos.append(i)
                    break

        return vhcl_nos
    
    def choosing_random_target(self, vhcl_no):

        # A random target is chosen from the route of the vehicle such that
        # the considered target is not to be necessarily covered by the considered
        # vehicle
        targ_options = [self.vhcl_tours[vhcl_no][i] for i in range(1, len(self.vhcl_tours[vhcl_no]) - 1)\
                        if self.vhcl_tours[vhcl_no][i] not in self._instance.targ_alloc_to_each_vhcl[vhcl_no]]
        if len(targ_options) == 0:
            return None
        else:
            return targ_options[random.randint(0, len(targ_options) - 1)]
        
    #------------Functions for savings computations
    def estimate_savings_remove_targ(self, vhcl_no, targ_removal):
        # In this function, the savings associated with removing a particular
        # target from a vehicle's tour is obtained
        
        # Obtaining vertices before and after considered target
        i = self.vhcl_tours[vhcl_no].index(targ_removal) # Obtaining index in which target is located
        t_prev = self.vhcl_tours[vhcl_no][i - 1]
        t_next = self.vhcl_tours[vhcl_no][i + 1]

        # Computing the savings
        return (self._instance.get_vhcl_cost(t_prev, targ_removal, vhcl_no)\
            + self._instance.get_vhcl_cost(targ_removal, t_next, vhcl_no)\
            - self._instance.get_vhcl_cost(t_prev, t_next, vhcl_no))
    
    def estimate_savings(self, vhcl_no, skip_targets = []):
        
        # In this function, the estimated savings obtained by removing targets
        # from the chosen vehicle is obtained. Note that for this purpose, targets
        # that are necessarily covered by the considered vehicle are excluded from
        # the list. Moreover, if there are some targets that need to be skipped, that
        # can be passed as well.
        assert(vhcl_no <= self._instance.get_num_vhcls())

        savings = {}
        # Running through all targets in the route. Note that each vehicle's route
        # starts and ends at the depot
        # Checking if the considered vehicle has a non-empty tour
        if len(self.vhcl_tours[vhcl_no]) == 0:
            return []

        for i in range(1, len(self.vhcl_tours[vhcl_no]) - 1):

            # Checking if chosen target needs to necessarily be covered by considered
            # vehicle or needs to be skipped.
            if self.vhcl_tours[vhcl_no][i] in self._instance.targ_alloc_to_each_vhcl[vhcl_no]\
                or self.vhcl_tours[vhcl_no][i] in skip_targets:
                continue
            else:

                # Obtaining vertices before and after considered target 
                # t_curr = self.vhcl_tours[vhcl_no][i]
                # t_prev = self.vhcl_tours[vhcl_no][i - 1]
                # t_next = self.vhcl_tours[vhcl_no][i + 1]

                # # Computing the savings
                # savings[t_curr] = self._instance.get_vhcl_cost(t_prev, t_curr, vhcl_no)\
                #     + self._instance.get_vhcl_cost(t_curr, t_next, vhcl_no)\
                #     - self._instance.get_vhcl_cost(t_prev, t_next, vhcl_no)
                savings[self.vhcl_tours[vhcl_no][i]] =\
                      self.estimate_savings_remove_targ(vhcl_no, self.vhcl_tours[vhcl_no][i])
                    
        # Sorting the savings dictionary in decreasing order, and returning that array
        return sorted(savings.items(), key = lambda x:x[1], reverse = True)
    
    # def savings_vehicle_removing_mult_targ(self, vhcl_no, targ_arr):
    #     # In this function, the savings associated with removing multiple targets
    #     # specified in targ_arr is obtained.

    #     # Obtaining vertices before and after considered target array
    #     # Obtaining index corresponding to the first target in targ_arr
    #     i = self.vhcl_tours[vhcl_no].index(targ_arr[0])
    #     t_prev = self.vhcl_tours[vhcl_no][i - 1]
    #     t_next = self.vhcl_tours[vhcl_no][i + len(targ_arr)]

    #     # Computing the savings for removing targ_arr
    #     # First, computing savings corresponding to removing edges connecting to
    #     # targ_arr
    #     savings = self._instance.get_vhcl_cost(t_prev, targ_arr[0], vhcl_no)\
    #         + self._instance.get_vhcl_cost(targ_arr[-1], t_next, vhcl_no)
    #     # Including savings corresponding to removing edges between targ_arr
    #     for j in range(len(targ_arr) - 1):
    #         savings += self._instance.get_vhcl_cost(targ_arr[j], targ_arr[j + 1], vhcl_no)
    #     # Removing cost associated with connecting t_prev and t_next
    #     savings -= self._instance.get_vhcl_cost(t_prev, t_next, vhcl_no)

    #     return savings 

    def savings_vehicle_removing_mult_targ(self, vhcl_no, targ_arr):
        # In this function, the savings associated with removing multiple targets
        # specified in targ_arr is obtained.

        # Making a copy of the route object
        route_copy = self.route_copy()

        # Computing savings by removing one target at a time
        savings = 0
        for t in targ_arr:
            # Obtaining the vertices before and after the considered target
            i = route_copy.vhcl_tours[vhcl_no].index(t)
            t_prev = route_copy.vhcl_tours[vhcl_no][i - 1]
            t_next = route_copy.vhcl_tours[vhcl_no][i + 1]
            # Updating the savings
            savings += route_copy._instance.get_vhcl_cost(t_prev, t, vhcl_no) \
                + route_copy._instance.get_vhcl_cost(t, t_next, vhcl_no)\
                - route_copy._instance.get_vhcl_cost(t_prev, t_next, vhcl_no)
            # Removing the considered target from the copied route object
            route_copy.vhcl_tours[vhcl_no].remove(t)
            # print('Targets before and after ', t, ' are ', t_prev, t_next)

        # # Obtaining the cost of the two edges that are incident to the target chunk
        # ini = self.vhcl_tours[vhcl_no].index(targ_arr[0])
        # fin = self.vhcl_tours[vhcl_no].index(targ_arr[-1])
        # t_before = self.vhcl_tours[vhcl_no][ini - 1]
        # t_after = self.vhcl_tours[vhcl_no][fin + 1]
        # print('Targets before and after, ', targ_arr, ' are ', t_before, ' and ', t_after,\
        #       ' respectively.')
        # cost_two_incident_edges = self._instance.get_vhcl_cost(t_before, targ_arr[0], vhcl_no)\
        #       + self._instance.get_vhcl_cost(targ_arr[-1], t_after, vhcl_no)

        return savings

    # def savings_vehicle_mult_targ_sorted(self, vhcl_no, num_targ):

    #     # In this function, the estimated savings obtained by removing multiple
    #     # targets from the chosen vehicle is obtained. Note that for this purpose, targets
    #     # that are necessarily covered by the considered vehicle are excluded from
    #     # the list.
    #     assert(vhcl_no <= self._instance.get_num_vhcls())
    #     # Checking if the considered vehicle has at least "num_targ" number of targets
    #     # that it covers. Note that in the vehicle's tour, the first and last
    #     # indeces are the depot.
    #     if len(self.vhcl_tours[vhcl_no]) < num_targ + 2:
    #         return []
        
    #     savings = {}
    #     for i in range(1, len(self.vhcl_tours[vhcl_no]) - num_targ):
    #         # Obtaining list of targets to be removed
    #         targ_arr = [self.vhcl_tours[vhcl_no][i + j] for j in range(num_targ)]
    #         # Checking if any of the targets in targ_arr is to be necessarily covered
    #         # by the vehicle
    #         if len([j for j in targ_arr if j in self._instance.targ_alloc_to_each_vhcl[vhcl_no]]) == 0:
                
    #             savings[tuple(targ_arr)] = self.savings_vehicle_removing_mult_targ(vhcl_no, targ_arr)

    #     # Sorting the savings dictionary in decreasing order, and returning that array
    #     return sorted(savings.items(), key = lambda x:x[1], reverse = True)
    
    def savings_vehicle_mult_targ_sorted(self, vhcl_no, num_targ):

        # In this function, the estimated savings obtained by removing multiple
        # targets from the chosen vehicle is obtained. Note that for this purpose, targets
        # that are necessarily covered by the considered vehicle are excluded from
        # the list.
        assert(vhcl_no <= self._instance.get_num_vhcls())
        
        savings = {}
        # Procuring the list of targets that can be removed
        targ_remov = [self.vhcl_tours[vhcl_no][i] for i in range(1, len(self.vhcl_tours[vhcl_no]) - 1)\
                      if self.vhcl_tours[vhcl_no][i] not in self._instance.targ_alloc_to_each_vhcl[vhcl_no]]
        # print('List of targets that can be removed from the maximal vehicle are ', targ_remov)
            
        # Checking if "targ_remov" has at least num_targ number of targets
        if len(targ_remov) < num_targ:
            return []
        
        # Running through the list of targets that can be removed
        for i in range(len(targ_remov) - num_targ + 1):
            # In this case, the targets to be removed are corresponding to
            # indeces i, i+1, ..., i+num_targ-1
            targ_arr = [targ_remov[i + j] for j in range(num_targ)]
            # print(targ_arr)

            savings[tuple(targ_arr)] = self.savings_vehicle_removing_mult_targ(vhcl_no, targ_arr)

        # Sorting the savings dictionary in decreasing order, and returning that array
        return sorted(savings.items(), key = lambda x:x[1], reverse = True)
    
    def removal_ratio_targets_mult_targ(self, max_vhcl_no, max_num_targ_chunk):
        # In this function, the ratio of cost of edges incident on the target
        # chunk considered to the cost of edges between the target array is obtained.
        # The intuition behind obtaining this ratio is to attempt to remove target
        # chunks from the maximal vehicle that are clustered and are far from the
        # rest of the vehicle's tour. Hence, the target chunks are sorted in the
        # decreasing order of this ratio

        removal_ratio = {}
        # Procuring the list of targets that can be removed
        targ_remov = [self.vhcl_tours[max_vhcl_no][i] for i in range(1, len(self.vhcl_tours[max_vhcl_no]) - 1)\
                      if self.vhcl_tours[max_vhcl_no][i] not in self._instance.targ_alloc_to_each_vhcl[max_vhcl_no]]        
        # print(targ_remov)

        # Running through different length chunks
        for length_chunk in range(2, max_num_targ_chunk + 1):

            # Running through the target chunks of given length
            for i in range(len(targ_remov) - length_chunk + 1):
                # In this case, the targets to be removed are corresponding to
                # indeces i, i+1, ... , i + length_chunk - 1
                targ_arr = [targ_remov[i + j] for j in range(length_chunk)]
                # print('Considered target array for removal is ', targ_arr)

                # Obtaining the average removal cost, and cost of removal of two edges incident on the target chunk
                removal_cost = sum([self._instance.get_vhcl_cost(targ_remov[i + j],\
                                                                 targ_remov[i + j + 1], max_vhcl_no)\
                                                                    for j in range(length_chunk - 1)])/(length_chunk - 1)
                # Obtaining the targets before and after the target chunk
                ini = self.vhcl_tours[max_vhcl_no].index(targ_arr[0])
                fin = self.vhcl_tours[max_vhcl_no].index(targ_arr[-1])
                t_before = self.vhcl_tours[max_vhcl_no][ini - 1]
                t_after = self.vhcl_tours[max_vhcl_no][fin + 1]
                cost_two_incident_edges = self._instance.get_vhcl_cost(t_before, targ_arr[0], max_vhcl_no)\
                      + self._instance.get_vhcl_cost(targ_arr[-1], t_after, max_vhcl_no)

                # Appending the ratio cost_two_edges/(savings - cost_two_edges), and the savings metric as well
                removal_ratio[tuple(targ_arr)] = cost_two_incident_edges/removal_cost

        # Sorting the savings dictionary in decreasing order, and returning that array
        return sorted(removal_ratio.items(), key = lambda x:x[1], reverse = True)
    
    
    #---------------------Functions for insertion
    def insertion_cost(self, targ, vhcl_no):
        # In this function, the estimated increase in tour cost with insertion
        # of considered target is obtained. Further, the position of insertion
        # is also returned

        pos_insertion = 0
        cost_insertion = np.infty
        # Running through the tour of the current vehicle
        # Checking if the vehicle has a non-zero tour
        if len(self.vhcl_tours[vhcl_no]) == 0:
            # In this case, the insertion cost is two times cost of travel between
            # the depot and the considered target
            cost_insertion = 2*self._instance.get_vhcl_cost(targ, self._instance.vhcl_depot[vhcl_no], vhcl_no)
        else:

            # Running through the tour and identifying best location for insertion
            for j in range(1, len(self.vhcl_tours[vhcl_no])):

                # Obtaining vertices before considered location at current location
                t_prev = self.vhcl_tours[vhcl_no][j - 1]
                t_next = self.vhcl_tours[vhcl_no][j]
                # Checking if t_prev = t_next, which occurs when vehicle is not
                # moving from the depot
                if t_prev == t_next:
                    insertion_cost_j = 2*self._instance.get_vhcl_cost(targ, t_next, vhcl_no)
                else:
                    insertion_cost_j = self._instance.get_vhcl_cost(t_prev, targ, vhcl_no)\
                        + self._instance.get_vhcl_cost(targ, t_next, vhcl_no)\
                        - self._instance.get_vhcl_cost(t_prev, t_next, vhcl_no)
                
                # Keeping track of minimum cost of insertion and location of insertion
                if insertion_cost_j < cost_insertion:
                    cost_insertion = insertion_cost_j
                    pos_insertion = j

        return cost_insertion, pos_insertion
    
    def vhcl_with_min_insertion_cost(self, targ, vhcl_no_max_cost):
        # In this function, the vehicle with the minimum cost of insertion is
        # obtained

        min_cost_insertion = np.infty
        vhcl_no_min_insertion = np.NaN
        pos_insertion = np.NaN

        # Running through all vehicles
        for i in self.vhcl_tour_costs.keys():

            if i == vhcl_no_max_cost: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue
            else:
                # Obtaining the cost of insertion and position of insertion for
                # considered vehicle
                cost_insertion_i, pos_insertion_i =\
                      self.insertion_cost(targ, i)
                if cost_insertion_i < min_cost_insertion:
                    min_cost_insertion = cost_insertion_i
                    vhcl_no_min_insertion = i
                    pos_insertion = pos_insertion_i

        return vhcl_no_min_insertion, min_cost_insertion, pos_insertion
    
    def vhcl_with_min_insertion_cost_for_swap(self, targ, vhcl_no_max_cost):
        # In this function, the vehicle with the minimum cost of insertion is
        # obtained for swapping. In this case, the vehicle with which the swap
        # needs to be performed needs to have at least one target to swap with.

        min_cost_insertion = np.infty
        vhcl_no_min_insertion = np.NaN
        pos_insertion = np.NaN

        # Running through all vehicles
        for i in self.vhcl_tour_costs.keys():

            if i == vhcl_no_max_cost: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue
            
            elif len(self.vhcl_tours[i]) < 3: # If vhcl tours is atleast 3, then it
            # is covering the depot, the target, and the depot, atleast.
                continue
            
            else:

                # Testing if there is at least one common target that the vehicle
                # covers
                flag = 0
                for j in range(1, len(self.vhcl_tours[i]) - 1):
                    if self.vhcl_tours[i][j] not in self._instance.targ_alloc_to_each_vhcl[i]:
                        flag = 1
                        break
                
                if flag == 1:
                    # Obtaining the cost of insertion and position of insertion for
                    # considered vehicle
                    cost_insertion_i, pos_insertion_i = self.insertion_cost(targ, i)
                    if cost_insertion_i < min_cost_insertion:
                        min_cost_insertion = cost_insertion_i
                        vhcl_no_min_insertion = i
                        pos_insertion = pos_insertion_i

        return vhcl_no_min_insertion, min_cost_insertion, pos_insertion
    
    def sorted_vhcls_from_least_insertion_cost(self, targ, vhcl_no_max_cost, n):
        # In this function, the vehicles are sorted in the increasing cost of
        # insertion cost. Further, the top "n" number of vehicles that have the
        # least cost of insertion are returned.

        insertion_cost_vhcls = {}
        pos_insertion = {}

        # Running through all vehicles
        for i in self.vhcl_tour_costs.keys():

            if i == vhcl_no_max_cost: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue
            else:
                # Obtaining the cost of insertion and position of insertion for
                # considered vehicle
                cost_insertion_i, pos_insertion_i = self.insertion_cost(targ, i)
                insertion_cost_vhcls[i] = cost_insertion_i
                pos_insertion[i] = pos_insertion_i

        # Sorting the vehicles in increasing order of insertion cost
        i = 0 # counter for number of vehicles to be considered
        sorted_insertion_cost = []
        sorted_pos_insertion = []
        sorted_vhcl_nos = []

        for (vhcl_no, inc) in sorted(insertion_cost_vhcls.items(), key=lambda x:x[1]):
            if i < n:
                sorted_insertion_cost.append((vhcl_no, inc))
                sorted_pos_insertion.append(pos_insertion[vhcl_no])
                sorted_vhcl_nos.append(vhcl_no)
                i += 1 # Incrementing the counter

        # pos_insertion_sorted = [(sorted_insertion_costs[i][0],\
        #                          pos_insertion[sorted_insertion_costs[i][0]]) for i in range(n)]
        
        # return [sorted_insertion_costs[i][0] for i in range(n)],\
        #       sorted_insertion_costs[:n], pos_insertion_sorted
        return sorted_vhcl_nos, sorted_insertion_cost, sorted_pos_insertion
    
    def sorted_vhcls_from_least_insertion_cost_for_swap(self, targ, vhcl_no_max_cost, n):
        # In this function, the vehicles are sorted in the increasing cost of
        # insertion cost for swap. Further, the top "n" number of vehicles that have the
        # least cost of insertion are returned.

        insertion_cost_vhcls = {}
        pos_insertion = {}

        # Running through all vehicles
        for i in self.vhcl_tour_costs.keys():

            if i == vhcl_no_max_cost: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue
            
            elif len(self.vhcl_tours[i]) < 3: # If vhcl tours is atleast 3, then it
            # is covering the depot, the target, and the depot, atleast.
                continue
            
            else:

                # Testing if there is at least one common target that the vehicle
                # covers
                flag = 0
                for j in range(1, len(self.vhcl_tours[i]) - 1):
                    if self.vhcl_tours[i][j] not in self._instance.vhcl_targ_allocation:
                        flag = 1
                        break

                if flag == 1:
                    # Obtaining the cost of insertion and position of insertion for
                    # considered vehicle
                    cost_insertion_i, pos_insertion_i = self.insertion_cost(targ, i)
                    insertion_cost_vhcls[i] = cost_insertion_i
                    pos_insertion[i] = pos_insertion_i

        # Sorting the vehicles in increasing order of insertion cost
        i = 0 # counter for number of vehicles to be considered
        sorted_insertion_cost = []
        sorted_pos_insertion = []
        sorted_vhcl_nos = []

        for (vhcl_no, inc) in sorted(insertion_cost_vhcls.items(), key=lambda x:x[1]):
            if i < n:
                sorted_insertion_cost.append((vhcl_no, inc))
                sorted_pos_insertion.append(pos_insertion[vhcl_no])
                sorted_vhcl_nos.append(vhcl_no)
                i += 1 # Incrementing the counter

        return sorted_vhcl_nos, sorted_insertion_cost, sorted_pos_insertion
    
    def vhcl_with_least_increase_tour_cost(self, targ, vhcl_no_max_cost):
        # In this function, the vehicle with the least tour cost with insertion
        # is chosen for insertion
        
        best_tour_cost_vhcl_except_maximal = np.infty
        vhcl_insertion = 0
        pos_insertion = 0
        for i in self.vhcl_tours.keys():
            if i != vhcl_no_max_cost:
                
                # Obtaining the insertion cost and position of insertion for considered vehicle
                insertion_cost_vhcl_i, pos_insertion_i = self.insertion_cost(targ, i)
                # Checking if considered vehicle has a better tour cost
                if self.vhcl_tour_costs[i] + insertion_cost_vhcl_i < best_tour_cost_vhcl_except_maximal:

                    best_tour_cost_vhcl_except_maximal = self.vhcl_tour_costs[i] + insertion_cost_vhcl_i
                    vhcl_insertion = i
                    pos_insertion = pos_insertion_i
                    
        return vhcl_insertion, best_tour_cost_vhcl_except_maximal, pos_insertion
    
    def sorted_vhcls_from_least_tot_tour_cost(self, targ, vhcl_no_max_cost, n):
        # In this function, the vehicles are sorted in the increasing cost of
        # total tour cost. Further, the top "n" number of vehicles that have the
        # least total cost of insertion are returned.
        
        tour_cost_vhcls = {}
        pos_insertion = {}

        # Running through all vehicles
        for i in self.vhcl_tour_costs.keys():

            if i == vhcl_no_max_cost: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue
            else:
                # Obtaining the cost of insertion and position of insertion for
                # considered vehicle. Further, the cost of the vehicle tour is
                # also obtained.
                cost_insertion_i, pos_insertion_i = self.insertion_cost(targ, i)
                tour_cost_vhcls[i] = self.vhcl_tour_costs[i] + cost_insertion_i
                pos_insertion[i] = pos_insertion_i
                
        # Sorting the vehicles in increasing order of insertion cost
        i = 0 # counter for number of vehicles to be considered
        sorted_total_cost = []
        sorted_pos_insertion = []
        sorted_vhcl_nos = []

        for (vhcl_no, inc) in sorted(tour_cost_vhcls.items(), key=lambda x:x[1]):
            if i < n:
                sorted_total_cost.append((vhcl_no, inc))
                sorted_pos_insertion.append(pos_insertion[vhcl_no])
                sorted_vhcl_nos.append(vhcl_no)
                i += 1 # Incrementing the counter
                
        return sorted_vhcl_nos, sorted_total_cost, sorted_pos_insertion
    
    def vhcl_i_insertion_sort_targets(self, targs, vhcl_no_for_insertion, vhcl_targs_remov):
        # In this function, the best target from "targs" to be inserted in the
        # considered vehicle's tour is identified. Further, the "targs" are sorted
        # in increasing order of the increase cost metric.
        
        increase_cost_metric_targs = {}
        # Running through each target and obtaining the increase in cost metric
        for t in targs:
            
            # Ensuring that considered target is not allocated through a vehicle-
            # target assignment
            if t in self._instance.targ_alloc_to_each_vhcl[vhcl_targs_remov]:
                
                continue
            
            cost, pos = self.insertion_cost(t, vhcl_no_for_insertion)
            increase_cost_metric_targs[(t, pos)] = cost
            
        # Sorting the dictionary in increasing order of increase cost metric and returning it
        return sorted(increase_cost_metric_targs.items(), key = lambda x:x[1])
    
    def insertion_cost_mult_targets(self, targ_arr, vhcl_no):
        # In this function, the estimated increase in tour cost with insertion of
        # considered target array is obtained. Further, the position of insertion
        # and orientation of insertion are also obtained. Note that orientation of
        # insertion represents whether the target array is inserted as it is or
        # in the flipped manner.
        # Note that since this function is considered for swap functions, wherein
        # multiple targets are swapped, the considered vehicle is presumed to
        # cover some targets at least.
        pos_insertion = 0
        cost_insertion = np.infty
        orientation = 0 # if orientation = 0, then insert as is; if 1, insert in
        # flipped manner. For example, if targ_arr = [1, 2, 3], we insert as is
        # if orientation is 0, whereas we insert it as [3, 2, 1] if orientation is 1.

        # Running through the considered vehicle's tour and identifying the best
        # location for insertion

        # Obtaining cost of edges corresponding to edges between the targets
        # of insertion
        insertion_cost_targ_edges = sum([self._instance.get_vhcl_cost(targ_arr[i], targ_arr[i + 1], vhcl_no)\
                                         for i in range(len(targ_arr) - 1)])
        edges = [(targ_arr[i], targ_arr[i + 1]) for i in range(len(targ_arr) - 1)]
        # print('Cost of insertion of edges between ', targ_arr, ' is computed using edges ', edges)

        for j in range(1, len(self.vhcl_tours[vhcl_no])):

            # Obtaining vertices before considered location at current location
            t_prev = self.vhcl_tours[vhcl_no][j - 1]
            t_next = self.vhcl_tours[vhcl_no][j]

            if len(targ_arr) == 1: # Checking if targ_arr has just one target or two targets
                orientation_arr = [0]
            else:
                orientation_arr = [0, 1]

            for ori_j in orientation_arr:
                if ori_j == 0:
                    insertion_cost_targ_j = insertion_cost_targ_edges\
                          + self._instance.get_vhcl_cost(t_prev, targ_arr[0], vhcl_no)\
                          + self._instance.get_vhcl_cost(targ_arr[-1], t_next, vhcl_no)
                else:
                    insertion_cost_targ_j = insertion_cost_targ_edges\
                          + self._instance.get_vhcl_cost(t_prev, targ_arr[-1], vhcl_no)\
                          + self._instance.get_vhcl_cost(targ_arr[0], t_next, vhcl_no)
                # Substracting cost associated with removing edge connecting t_prev and
                # t_next
                insertion_cost_targ_j -= self._instance.get_vhcl_cost(t_prev, t_next, vhcl_no)
    
                # Checking if obtained location of insertion is better than previous one
                if insertion_cost_targ_j < cost_insertion:
                    cost_insertion = insertion_cost_targ_j
                    pos_insertion = j
                    orientation = ori_j

        return cost_insertion, pos_insertion, orientation
    
    def insertion_mult_targets_recursive(self, targ_arr, vhcl_no):
        # In this function, the estimated increase in tour cost with insertion of
        # considered target array is obtained. To this end, explicit insertion is
        # considered, wherein each target in targ_arr can be inserted at any
        # position in the tour.

        # In this case, instead of keeping track of position of subsequent insertion
        # of each target is kept track of
        pos_insertion_arr = []
        cost_insertion = 0

        # Making a copy of the route object
        route_copy = self.route_copy()

        # Running through each target in targ_arr, and inserting the targets one
        # at a time
        for t in targ_arr:

            # Identifying the best position for inserting t, and the cost of insertion
            c, p = route_copy.insertion_cost(t, vhcl_no)
            # Inserting the target t at the obtained location, and updating the
            # insertion cost and position of insertion array
            cost_insertion += c
            pos_insertion_arr.append(p)
            route_copy.vhcl_tours[vhcl_no].insert(p, t)
            # print('Vehicle tour after inserting ', t, ' at position ', p, ' with cost ', c,\
            #       ' is ', route_copy.vhcl_tours[vhcl_no])
            
        # print('Cost of insertion and position of insertion for ', targ_arr, ' are ', cost_insertion, pos_insertion_arr)

        return cost_insertion, pos_insertion_arr

    
    def vhcl_with_min_insertion_cost_for_swap_mult_targ(self, targ_arr, vhcl_no_max_cost):
        # In this function, the vehicle with the minimum cost of insertion is obtained
        # for swapping multiple targets. In this case, the vehicle with which the swap 
        # needs to be performed needs to have at least len(targ_arr) - 1 number of
        # common targets that it covers to swap with.
        # Note that for the insertion, the targ_arr can be inserted as is or
        # in the opposite order. This is kept track of using the "orientation" variable.

        min_cost_insertion = np.infty
        vhcl_no_min_insertion = np.NaN
        min_pos_insertion = np.NaN
        min_orientation = np.NaN

        # Running through all vehicles
        for i in self.vhcl_tour_costs.keys():

            # print('Vehicle considered is ', i)            

            if i == vhcl_no_max_cost: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue
            
            elif len(self.vhcl_tours[i]) < len(targ_arr) + 1:
                continue # In this case, vehicle covers lesser number of targets
                # than required for a swap.

            else:

                # # Testing if there is atleast one set of common targets that the
                # # vehicle covers
                # for j in range(1, len(self.vhcl_tours[i]) - len(targ_arr)):
                #     flag = 0
                #     for k in range(len(targ_arr)):
                #         if self.vhcl_tours[i][j + k] in self._instance.targ_alloc_to_each_vhcl[i]:
                #             flag = 1
                #             break
                #     if flag == 0: # If no target in the considered set of targets is
                #         # to be necessarily covered by the vehicle, we are done
                #         break

                # Obtaining list of common targets covered by considered vehicle
                targ_arr_swap = [self.vhcl_tours[i][k] for k in range(1, len(self.vhcl_tours[i]) - 1)\
                                 if self.vhcl_tours[i][k] not in self._instance.targ_alloc_to_each_vhcl[i]]

                if len(targ_arr_swap) >= len(targ_arr) - 1:
                    # Obtaining the cost of insertion, position of insertion, and
                    # orientation for the considered vehicle
                    cost_insertion, pos_insertion, orientation =\
                          self.insertion_cost_mult_targets(targ_arr, i)

                    if cost_insertion < min_cost_insertion:
                        min_cost_insertion = cost_insertion
                        vhcl_no_min_insertion = i
                        min_pos_insertion = pos_insertion
                        min_orientation = orientation

        return vhcl_no_min_insertion, min_cost_insertion, min_pos_insertion, min_orientation
    
    def vhcl_with_min_insertion_cost_mult_targ_recursive(self, targ_arr, vhcl_no_max_cost):
        # In this function, the vehicle with the miniumum cost of insetion is obtained
        # for swapping multiple targets. In this case, the vehicle with which the swap
        # needs to be performed needs to have at least one common target that it covers
        # to swap with. Note that for the insertion, the targets in target_arr is inserted
        # one target at a time.

        min_cost_insertion = np.infty
        vhcl_no_min_insertion = np.NaN
        min_pos_insertion_arr = []

        # Running through all vehicles
        for i in self.vhcl_tour_costs.keys():

            if i == vhcl_no_max_cost: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue

            else:

                # Obtaining list of common targets covered by considered vehicle
                targ_arr_swap = [self.vhcl_tours[i][k] for k in range(1, len(self.vhcl_tours[i]) - 1)\
                                 if self.vhcl_tours[i][k] not in self._instance.targ_alloc_to_each_vhcl[i]]
                # print('List of targets that can be removed from vehicle', i, ' are ', targ_arr_swap)
                
                if len(targ_arr_swap) >= 1:

                    # Obtaining the cost of insertion, position array for insertion
                    cost_insertion, pos_arr_insertion = self.insertion_mult_targets_recursive(targ_arr, i)
                    # print('Cost of insertion and position array for insertion of ', targ_arr, ' is ',\
                    #       cost_insertion, pos_arr_insertion)

                    if cost_insertion < min_cost_insertion:
                        min_cost_insertion = cost_insertion
                        vhcl_no_min_insertion = i
                        min_pos_insertion_arr = pos_arr_insertion

        return vhcl_no_min_insertion, min_cost_insertion, min_pos_insertion_arr
                
    
    # def vhcl_i_mult_insertion_sort_targets(self, targs, num_targs, vhcl_no_for_insertion, vhcl_targs_remov):
    #     # In this function, the best target array from "targs" to be inserted in the considered
    #     # vehicle's tour is identified. Further, the target arrays are sorted in increasing
    #     # order of the increase cost metric.

    #     increase_cost_metric_targ_array = {}
    #     # Running through each target array and obtaining the increase in cost metric
    #     for i in range(len(targs) - num_targs + 1):

    #         # Obtaining the target array, and including only those targets that
    #         # are not pre-allocated. NOTE THAT len(targ_arr) = num_targs iff none
    #         # of the targets in the considered chunk are pre-allocated.
    #         targ_arr = [targs[i + j] for j in range(num_targs)\
    #                      if targs[i + j] not in self._instance.targ_alloc_to_each_vhcl[vhcl_targs_remov]]
            
    #         if len(targ_arr) == num_targs: # Checking if none of the targets in considered
    #             # target chunk from i to i + num_targs are pre-allocated
    #             increase_cost_metric_targ_array[tuple(targ_arr)] =\
    #                   self.insertion_cost_mult_targets(targ_arr, vhcl_no_for_insertion)[0]
                
    #     # Sorting the dictionary in increasing order of increase cost metric and returning it
    #     return sorted(increase_cost_metric_targ_array.items(), key = lambda x:x[1])
    
    def vhcl_i_mult_insertion_sort_targets(self, targ_arr_inserted, num_targs,\
                                            vhcl_no_for_insertion, vhcl_targs_remove):
        # In this function, the chunks of targets to be removed from the vehicle
        # "vhcl_targs_remove" and inserted to "vhcl_no_for_insertion" are sorted.
        # The size of the chunks of targets considered is num_targs or num_targs - 1.
        # The target arrays are sorted in increasing order of the increase cost
        # metric.

        increase_cost_metric_targ_array = {}
        # Obtaining the list of targets that can be removed, which are targets that
        # have not just been inserted (specified in targ_arr_inserted) from the maximal
        # vehicle to the considered vehicle, and targets that are pre-allocated to
        # the considered vehicle.
        targ_arr_remove = []
        for k in range(1, len(self.vhcl_tours[vhcl_targs_remove]) - 1):
            if self.vhcl_tours[vhcl_targs_remove][k] not in self._instance.targ_alloc_to_each_vhcl[vhcl_targs_remove]\
            and self.vhcl_tours[vhcl_targs_remove][k] not in targ_arr_inserted:
                targ_arr_remove.append(self.vhcl_tours[vhcl_targs_remove][k])
        # print('Available list of targets, after removing necessary targets ',\
        #       self._instance.targ_alloc_to_each_vhcl[vhcl_targs_remove], 'is ', targ_arr_remove)

        # Running through each chunk of targets and obtaining the increase in cost metric.
        # Note that the size of the chunk of targets is num_targs or num_targs - 1
        for i in range(len(targ_arr_remove) - num_targs + 1):

            # Generating the two sets of chunks of targets, of length num_targs - 1
            # and num_targs, respectively.
            targ_chunks = []

            targ_chunk = [targ_arr_remove[i + j] for j in range(num_targs)]
            targ_chunks.append(targ_chunk)
            targ_chunks.append(targ_chunk[0:-1]) # In this chunk, the last target is not considered.
            # Hence, the chunk if of size num_targ - 1.

            # If i == len(targ_arr_remove) - num_targs, then one more chunk is
            # considered, which corresponds the last chunk of size num_targs - 1.
            if i == len(targ_arr_remove) - num_targs:
                targ_chunks.append(targ_chunk[1:]) # In this chunk, the first target is not considered.

            # FOR EXAMPLE, IF TARG_ARR_REMOVE IS [1, 2, 3], AND NUM_TARGS = 2, THEN
            # WE CONSIDER THE CHUNKS [1], [2], [3], [1, 2], [2, 3].

            # Obtaining the increase in cost metric, and position and orientation of
            # insertion for each chunk
            for targ_chunk_insert in targ_chunks:
                cost_insertion, pos_insertion, orientation =\
                          self.insertion_cost_mult_targets(targ_chunk_insert, vhcl_no_for_insertion)
                # Appending to the dictionary
                increase_cost_metric_targ_array[tuple(targ_chunk_insert), pos_insertion,\
                                                orientation] = cost_insertion
                
        # print('List of target chunks considered for insertion is ', increase_cost_metric_targ_array)
                    
        # Sorting the dictionary in increasing order of increase cost metric and returning it
        return sorted(increase_cost_metric_targ_array.items(), key = lambda x:x[1])
    
    def vhcl_i_savings_insertion_ratio_mult_sort_targets(self, targ_arr_inserted, max_targ_chunk_length,\
                                                         vhcl_no_for_insertion, vhcl_targs_remove):
        # In this function, the chunks of targets to be removed from the vehicle
        # "vhcl_targs_remove" and inserted to "vhcl_no_for_insertion" are sorted.
        # The largest size of the chunks of targets considered is max_targ_chunk_length.
        # The target arrays are sorted in descending order of ratio of savings for
        # removal for "vhcl_targs_remove" and insertion cost for "vhcl_no_for_insertion".

        ratio_savings_insertion_targ_array = {}
        # Obtaining the list of targets that can be removed, which are targets that
        # have not just been inserted (specified in targ_arr_inserted) from the maximal
        # vehicle to the considered vehicle, and targets that are pre-allocated to
        # the considered vehicle.
        targ_arr_remove = []
        
        for k in range(1, len(self.vhcl_tours[vhcl_targs_remove]) - 1):
            if self.vhcl_tours[vhcl_targs_remove][k] not in self._instance.targ_alloc_to_each_vhcl[vhcl_targs_remove]\
            and self.vhcl_tours[vhcl_targs_remove][k] not in targ_arr_inserted:
                targ_arr_remove.append(self.vhcl_tours[vhcl_targs_remove][k])
                
        # print('Targets that can be removed from the other vehicle are ', targ_arr_remove)

        # Running through each chunk of targets for length varying from one to
        # max_targ_chunk_length
        for i in range(1, max_targ_chunk_length + 1):

            # Running through chunks of size i
            for j in range(len(targ_arr_remove) - i + 1):

                # Obtaining the chunk of targets considered
                targ_chunk = [targ_arr_remove[j + k] for k in range(i)]

                # Obtaining the insertion cost into the other vehicle, and the
                # savings in the vhcl considered for removing the chunk of targets
                insert, pos_arr_insertion = self.insertion_mult_targets_recursive(targ_chunk, vhcl_no_for_insertion)
                saving = self.savings_vehicle_removing_mult_targ(vhcl_targs_remove, targ_chunk)

                # Checking if the insertion cost is nearly zero
                if insert <= 10**(-8):                    
                    ratio_savings_insertion_targ_array[(tuple(targ_chunk), tuple(pos_arr_insertion), insert, saving)]\
                          = np.infty                          
                else:                    
                    ratio_savings_insertion_targ_array[(tuple(targ_chunk), tuple(pos_arr_insertion), insert, saving)]\
                          = saving/insert
                          
                # print('Target chunk considered is ', targ_chunk, '. Saving and insertion cost are ',\
                #       saving, insert)

        return sorted(ratio_savings_insertion_targ_array.items(), key = lambda x:x[1], reverse = True)

    def estimated_savings_mult_targets(self, vhcl_no, num_targets):
        # In this function, the estimated savings obtained by removing multiple
        # targets from a vehicle's tour is obtained.
        # It should be noted that when removing multiple targets, the order of
        # removal does not matter. We use that fact to remove targets one at a time

        assert(vhcl_no <= self._instance.get_num_vhcls())

        savings = {}
        # Running through all combinations of targets in the route. Note that each 
        # vehicle's route starts and ends at the depot
        # Checking if the considered vehicle has a non-empty tour
        if len(self.vhcl_tours[vhcl_no]) == 0:
            return []
        
        # Obtaining list of indeces corresponding to targets covered, which excludes targets that need to be
        # necessarily covered by the considered vehicle
        # T = [t for t in self.vhcl_tours[vhcl_no][1:-1] if t not in self._instance.targ_alloc_to_each_vhcl[vhcl_no]]
        T_indeces = [i for i in range(1, len(self.vhcl_tours[vhcl_no]) - 1) if self.vhcl_tours[vhcl_no][i]\
                      not in self._instance.targ_alloc_to_each_vhcl[vhcl_no]]
        # Checking if there are atleast "num_targets" number of common targets the vehicle
        # covers
        if len(T_indeces) < num_targets:
            return []
               
        # Running through all possible combinations
        for comb_targ in combinations(T_indeces, num_targets):
            # Initializing the savings corresponding to the selected combination
            savings_comb = 0
            # Making a copy of the vehicle tour
            vhcl_tour = self.vhcl_tours[vhcl_no]
            
            # Running through each index corresponding to each target in the combination
            for i in range(len(comb_targ)):

                # Obtaining the previous and next vertices
                t_prev = vhcl_tour[comb_targ[i] - i - 1]
                t_curr = vhcl_tour[comb_targ[i] - i]
                t_next = vhcl_tour[comb_targ[i] - i + 1]
                print('Previous and next vertices for target ', self.vhcl_tours[vhcl_no][comb_targ[i]],\
                      ' are ', t_prev, ' and ', t_next, '.')
                # Updating the savings corresponding to the combination
                savings_comb += self._instance.get_vhcl_cost(t_prev, t_curr, vhcl_no)\
                    + self._instance.get_vhcl_cost(t_curr, t_next, vhcl_no)\
                    - self._instance.get_vhcl_cost(t_prev, t_next, vhcl_no)
                # Removing the current target from "vhcl_tour"
                vhcl_tour.remove(t_curr)

            # Appending the obtained savings to the array
            savings[tuple([self.vhcl_tours[vhcl_no][i] for i in comb_targ])] = savings_comb

        # Sorting the savings dictionary in decreasing order, and returning that array
        return sorted(savings.items(), key = lambda x:x[1], reverse = True)
    
    def obtain_targets_not_in_tour(self, vhcl_no):
        # In this function, the list of targets that are not in the considered vehicle's
        # tour are obtained. Note that the obtained list of targets are such that
        # no target is pre-assigned to another vehicle.
        targets_not_in_tour = []

        # Running through each vehicle
        for i in self.vhcl_tour_costs.keys():

            if i == vhcl_no: # Skipping if considered vehicle is same
                # as vehicle with the maximum tour cost
                continue

            for j in range(1, len(self.vhcl_tours[i]) - 1):
                if self.vhcl_tours[i][j] not in self._instance.vhcl_targ_allocation:
                    targets_not_in_tour.append((self.vhcl_tours[i][j], i, j))

        return targets_not_in_tour
    
    def get_targets_vicinity(self, targ, targets_not_in_tour, num_targ_neighbours):
        # In this function, for a given target "targ", the
        # closed "num_targ_neighbours" number of targets are returned. For this purpose,
        # the targets that are not present in the tour of the considered vehicle that
        # are also not pre-assigned to another vehicle are obtained. Note that the
        # list of targets that are not in the considered vehicle's tour and are not
        # pre-assigned to another vehicle are already provided in "targets_not_in_tour".

        targets_vicinity = {}
        # Running through each target in target_not_in_tour
        for (t, vhcl_ind, pos) in targets_not_in_tour:

            # Obtaining the Euclidean distance between targ and t
            targets_vicinity[(t, vhcl_ind, pos)] = self._instance.get_targ_dist(targ, t)
        
        # Sorting the dictionary in increasing order, and returning the first num_targ_neighbours
        # number of targets
        # print(sorted(targets_vicinity.items(), key=lambda x:x[1])[:num_targ_neighbours])
        return sorted(targets_vicinity.items(), key=lambda x:x[1])[:num_targ_neighbours]

    def two_opt_edges(self, vhcl_no):
        # In this function, a two-opt swap of edges is performed for the considered vehicle.

        # Checking if for the considered vehicle, the number of vertices covered is
        # greater than three; only under this case, a two-opt swap can be performed.
        # We consider greater than 4 since if the vehicle has four vertices, then the
        # first and last vertices are the depot. No improvement will be obtained from
        # two-opt swap since the costs are symmetric.
        if len(self.vhcl_tours[vhcl_no]) > 4:
            # for i in range(len(self.vhcl_tours[vhcl_no]) - 3):
            #     # Removing the edges between i and i+1, and i+2 and i+3, and connecting
            #     # edges i and i+2, and i+1 and i+3.
            #     city_1 = self.vhcl_tours[vhcl_no][i]; city_2 = self.vhcl_tours[vhcl_no][i+1];
            #     city_3 = self.vhcl_tours[vhcl_no][i+2]; city_4 = self.vhcl_tours[vhcl_no][i+3];
            #     cost_diff = self._instance.get_vhcl_cost(city_1, city_2, vhcl_no)\
            #         + self._instance.get_vhcl_cost(city_3, city_4, vhcl_no)\
            #         - self._instance.get_vhcl_cost(city_1, city_3, vhcl_no)\
            #         - self._instance.get_vhcl_cost(city_2, city_4, vhcl_no)
                
            #     print(cost_diff, city_1, city_2, city_3, city_4)
            #     if cost_diff >= 10**(-6):

            #         print('Better solution after 2-opt swap obtained. Swapping ', city_1, city_2, city_3, city_4)
            #         self.vhcl_tours[vhcl_no][i + 1] = city_3
            #         self.vhcl_tours[vhcl_no][i + 2] = city_2
            #         # Updating the tour cost
            #         self.set_vhcl_i_tour_cost(self.vhcl_tour_costs[vhcl_no] - cost_diff, vhcl_no)
            i = 0
            flag = 0 # flag variables to keep track of whether an improved solution
            # is obtained.
            
            while i < len(self.vhcl_tours[vhcl_no]) - 3:
                # Removing the edges between i and i+1, and i+2 and i+3, and connecting
                # edges i and i+2, and i+1 and i+3.
                city_1 = self.vhcl_tours[vhcl_no][i]; city_2 = self.vhcl_tours[vhcl_no][i+1];
                city_3 = self.vhcl_tours[vhcl_no][i+2]; city_4 = self.vhcl_tours[vhcl_no][i+3];
                cost_diff = self._instance.get_vhcl_cost(city_1, city_2, vhcl_no)\
                    + self._instance.get_vhcl_cost(city_3, city_4, vhcl_no)\
                    - self._instance.get_vhcl_cost(city_1, city_3, vhcl_no)\
                    - self._instance.get_vhcl_cost(city_2, city_4, vhcl_no)
                
                # print(cost_diff, city_1, city_2, city_3, city_4)
                if cost_diff >= 10**(-6):

                    # print('Better solution after 2-opt swap obtained. Swapping ', city_1, city_2, city_3, city_4)
                    self.vhcl_tours[vhcl_no][i + 1] = city_3
                    self.vhcl_tours[vhcl_no][i + 2] = city_2
                    if i > 0:
                        i -= 1 # In this case, we decrease i, to check if we can
                        # do one more swap.
                        flag = 1
                    else:
                        i += 1 # If i was zero, we just go forward as is.
                        flag = 0
                    # Updating the tour cost
                    # print('Vehicle tour cost before swap was ', self.vhcl_tour_costs[vhcl_no])
                    self.set_vhcl_i_tour_cost(self.vhcl_tour_costs[vhcl_no] - cost_diff, vhcl_no)
                    # print('Vehicle tour cost after swap is ', self.vhcl_tour_costs[vhcl_no])
                    
                elif flag == 1:
                    
                    # If previously, we had an improved solution and had decreased
                    # i by 1, but now, no improvement is obtained, we increase i by 2
                    # to avoid exploring a chunk already explored.                    
                    i += 2
                    flag = 0
                    
                else:
                    
                    i += 1
                    flag = 0

    def strictly_better_than(self, other):
        return self.route_cost < other.route_cost

    def slightly_worse_than(self, other):
        return percent_change(self.route_cost, other.route_cost) <= 5

    def far_from(self, other):
        """re-centering criteria"""
        return percent_change(self.route_cost, other.route_cost) >= 20