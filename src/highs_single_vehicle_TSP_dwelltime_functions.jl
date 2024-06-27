# In this function, a single vehicle TSP is solved using exact algorithm or
# using LKH.

#using JuMP, Gurobi, LinearAlgebra, Random, Plots; pythonplot()
using JuMP, LinearAlgebra, Random, Plots
include("instance.jl")
include("callback_functions_lazy_cuts.jl")
include("running_LKH.jl")

#constraint_lazycallback()
#cuts_LP_lazycallback()

import MathOptInterface as MOI

function single_vehicle_TSP_model(instance::AbstractInstance, vertices)
    # Here, instance corresponds to the instance object corresponding to the
    # considered graph. Further, vertices represents the array of vertices covered 
    # by vehicle number "vhcl_no". It should be noted that this includes the depot
    # of the vehicle as well.

    # Initializing the model (without the subtour constraints) 
    #model = Model(Gurobi.Optimizer)
    model = Model(HiGHS.Optimizer)
    MOI.set(model, MOI.Silent(), true)

    # Defining binary variables for the edges
    @variable(model, x[i in vertices, j in vertices; i < j], Bin)

    # Adding the degree constraint
    @constraint(model, c[j in vertices], sum(x[i, j] for i in vertices if i < j)
     + sum(x[j, i] for i in vertices if i > j) == 2)

    # Adding the objective function
    @objective(model, Min, sum(instance.cost_traversal[min(i, j)][max(i, j)]*x[min(i, j), max(i, j)]
     for (i, j) in combinations(vertices, 2)))

    # print(model)
    return model

end

function single_vehicle_TSP_dwell_times(instance::AbstractInstance, 
                                                targ_covered, 
                                                vhcl_no=1,
                                                exact=true;
                                                beta1=.2,
                                                beta2=.5,
                                                tolerance=.0001,
                                                initial=nothing
                                                )
    # solves the TSP problem and the dwell times for a single vehicle
    # uses single_vehicle_TSP_exact to solve the tour
    # assumes targ_covered is an array of POI keys 
    #       also that targ_covered does not include depots
    # exact is a boolean, if true, we find exact tour 
    #   if false, we just use LKH
    # beta1 and beta2 are used for gradient descent
    # returns tour, dwell times, tour cost, and key_array
    #       key_array should be redundant, it is returned since targ_covered is changed in single_vehicle_TSP_exact
    #       dwell times are in the same order as targ_covered, and in same order as the return key_array
    key_array = [key for key in targ_covered]
    taui_arr = [instance.tau for key in key_array]
    if exact
        tour,tour_cost = single_vehicle_TSP_exact(instance,targ_covered,vhcl_no)
    else
        tour=single_vehicle_TSP_LKH(instance,targ_covered,vhcl_no)
        tour_cost=path_cost(instance,tour)
    end
    dwell_times=nesterov_gradient_descent(taui_arr,instance.alpha,1.,initial=initial,tolerance=tolerance)
    return tour, dwell_times, tour_cost, key_array
end

function path_cost(instance,tour)
    # returns path cost of tour, assumes it starts and ends with same node
    # path_cost(Instance, [1,2,3,1])
    path_cost=0
    for i in range(1,length(tour)-1)
        path_cost+=instance.cost_traversal[tour[i]][tour[i+1]]
    end
    return path_cost
end
function single_vehicle_TSP_LKH(instance::AbstractInstance, targ_covered, vhcl_no = 1)
    # returns tour obtained from LKH
    insert!(targ_covered, 1, collect(keys(instance.depots))[vhcl_no])
    loop=construct_tour_LKH(instance,targ_covered)
    if length(loop)<=3
        return loop
    end

    edges_tour = [(min(loop[i], loop[i + 1]), max(loop[i], loop[i + 1])) for i in 1: length(loop) - 1]
    edges_traversed = Edge.(edges_tour)
    G_soln = Graphs.SimpleGraph(edges_traversed)
    # Obtaining the cycle starting at the depot
    cycle = cycle_basis(G_soln, collect(keys(instance.depots))[vhcl_no])[1]
    # Inserting node 1 in the beginning, since the rest of the cycle would have been returned
    insert!(cycle, 1, collect(keys(instance.depots))[vhcl_no])
    return cycle    
end
function single_vehicle_TSP_exact(instance::AbstractInstance, targ_covered, vhcl_no = 1)
    # In this function, the integer program formulation for the single vehicle
    # TSP is solved using lazy cuts.

    global model
    global T
    # Obtaining new set of vertices to be covered including the depot of the considered
    # vehicle
    insert!(targ_covered, 1, collect(keys(instance.depots))[vhcl_no])
    if length(targ_covered)<4
        push!(targ_covered,collect(keys(instance.depots))[vhcl_no])
        return targ_covered,path_cost(instance,targ_covered)
    end
    T = targ_covered
    # println(T)

    # Obtaining the model containing the integer program formulation
    model = single_vehicle_TSP_model(instance, T)
    # print(model)

    # Warm starting the model using LKH
    # Obtaining the tour from LKH
    tour = construct_tour_LKH(instance, T)
    # println(tour)
    # Obtaining the list of edges in the tour
    edges_tour = [(min(tour[i], tour[i + 1]), max(tour[i], tour[i + 1])) for i in 1: length(tour) - 1]
    # println(edges_tour)
    # Running through all edges in the model
    for e in combinations(T, 2)
        if (min(e[1], e[2]), max(e[1], e[2])) in edges_tour
            set_start_value(model[:x][min(e[1], e[2]), max(e[1], e[2])], 1)
            # println("Setting ", [min(e[1], e[2]), max(e[1], e[2])], " to 1")
        else
            set_start_value(model[:x][min(e[1], e[2]), max(e[1], e[2])], 0)
        end
    end

    # Adding lazy cuts
    #MOI.set(model, MOI.RawOptimizerAttribute("LazyConstraints"), 1)
    #set_attribute(model, MOI.LazyConstraintCallback(), constraint_lazycallback())
    #set_attribute(model, MOI.UserCutCallback(), cuts_LP_lazycallback())
    optimize!(model)

    # Retrieving the solution
    # Reading the optimal solution and returning the tour
    # Running through all variables in the model to obtain the tour
    arr_edges_traversed = [(e[1], e[2]) for e in combinations(T, 2)
     if value(model[:x][min(e[1], e[2]), max(e[1], e[2])]) > 0.5]
    edges_traversed = Edge.(arr_edges_traversed)
    # println(edges_traversed)
    G_soln = Graphs.SimpleGraph(edges_traversed)
    # Obtaining the cycle starting at the depot
    cycle = cycle_basis(G_soln, collect(keys(instance.depots))[vhcl_no])[1]
    # Inserting node 1 in the beginning, since the rest of the cycle would have been returned
    insert!(cycle, 1, collect(keys(instance.depots))[vhcl_no])

    return cycle, objective_value(model)

end

function Ii(dwell_time,tau)
    # computing each Ii function
    if dwell_time<0
        return Inf
    end
    Pi = 1 - 0.5*exp(-sqrt(dwell_time/tau))
    Ii=Pi*log(Pi) + (1 - Pi)*log(1 - Pi) + log(2)

    # Checking if Ii is NaN, which occurs when Pi is 1. This occurs when the dwell
    # time is very large in comparison to tau, which makes information gain zero.
    if isnan(Ii)
        return Inf
    end
    return Ii
end

function undiscounted_info_gain_func(dwell_time_arr, taui_arr)
    # Computing the "G" function
    G = 0
    for i in range(1, length(dwell_time_arr))
        G+=Ii(dwell_time_arr[i],taui_arr[i])
    end

    # Checking if G is NaN, which occurs when Pi is 1. This occurs when the dwell
    # time is very large in comparison to tau, which makes information gain zero.
    if isnan(G)
        return Inf
    end
    
    return G
end

function dwell_time_info_gain_func(dwell_time_arr, taui_arr, alpha)
    # In this function, the negative log of the information gain function value
    # is obtained given the dwell time array, which contains the dwell time of
    # all targets, the tau values for each target corresponding to sensitivity
    # of time spent on information gain, and the alpha parameter, which corresponds
    # to discount information gain

    # # Ensuring the length of the dwell time array and tau array are equal to the
    # # number of targets
    # @assert length(dwell_time_arr) == num_targ
    # @assert length(taui_arr) == num_targ

    # Computing the "G" function
    G = undiscounted_info_gain_func(dwell_time_arr,taui_arr)

    # Checking if G is NaN, which occurs when Pi is 1. This occurs when the dwell
    # time is very large in comparison to tau, which makes information gain zero.
    if isnan(G)
        return Inf
    end
    
    return alpha*(sum(dwell_time_arr)) - log(G)
    
end


function gradient_dwell_time_func(dwell_time_arr, taui_arr, alpha,zero_value=-1)
    # In this function, the gradient of the negative log of the information
    # gain function is computed.
    
    # negated this since we are flipping the function and doing minimization
    # grad of -log(J)=-(-a(d_1+...+d_n)+ln(G))
    # d/(d di) (a(d_1+...+d_n)-ln(G))
    # a - (G')/G
    # a - (Ii')/G  since G is a sum of Iis, and (d Ij)(d di)=0 for i=/=j
    # new Ii' derivative is 
    # (1/(4*sqrt(d_i tau_i)))*e^(-sqrt(d_i/tau_i))*(ln(P_i)-ln(1-P_i))
    # since (1-P_i)=e^(-sqrt(d_i/tau_i))/2
    # (1/(2*sqrt(d_i tau_i)))*(1-P_i)*(ln(P_i)-ln(1-P_i))
    

    # Initializing the gradient value of the function at given point
    grad = Array{Float64}(undef, length(dwell_time_arr))
    
    # Computing the value of the G function
    G = 0
    for i in range(1, length(dwell_time_arr))
        Pi = 1 - 0.5*exp(-sqrt(dwell_time_arr[i]/taui_arr[i]))
        # println("Pi for i = ", i, " is ", Pi)
        # println("RHS for G updation is ", Pi*log(Pi) + (1 - Pi)*log(1 - Pi) + log(2))
        G += Pi*log(Pi) + (1 - Pi)*log(1 - Pi) + log(2)
    end

    # println("G value is ", G)
    # Computing the gradient
    for i in range(1, length(dwell_time_arr))
        Pi = 1 - 0.5*exp(-sqrt(dwell_time_arr[i]/taui_arr[i]))
        if dwell_time_arr[i]<=0
            grad[i] = zero_value
        else
            grad[i] = alpha - (1/G)*(1/(2*sqrt(dwell_time_arr[i]*taui_arr[i])))*
                                (1-Pi)*
                                (log(Pi) - log(1 - Pi))
        end
    end
    # println("Gradient is ", grad)

    return grad

end

function hessian_test(dwell_time_arr, taui_arr)
    # In this function, the Hessian is constructed for the given function.
    # NOTE THAT THIS FUNCTION SHOULD BE CALLED ONLY WHEN ALL DWELL TIMES ARE zero

    @assert(all(y->y>0, dwell_time_arr))

    # Obtaining all eigenvalues, and checking if any of the eigenvalues is less
    # than or equal to zero
    # Computing the G function

    # since d^2 -log(J)/((d di)(d dj))=0 for i =/= j, this is a diagonal matrix
    # the eigenvalues are simply d^2 -log(J)/(d di)^2
    # this is (d/d di) a-(Ii')/G (look in new_gradient_dwell_time_func)
    # -(G*Ii''-Ii'(dG/d di))/(G^2)
    # Ii'(d Ii/(d di))/G^2 - Ii''/G
    # (Ii'/G)^2 - Ii''/G

    
    # Ii'=(1/(2*sqrt(d_i tau_i)))*(1-P_i)*(ln(P_i)-ln(1-P_i))
    # Ii''=((1-P_i)/(4 d_i tau_i))*(1/P_i-(ln(P_i)-ln(1-P_i))(1+sqrt(tau_i/d_i)))

    G = 0
    for i in range(1, length(dwell_time_arr))
        Pi = 1 - 0.5*exp(-sqrt(dwell_time_arr[i]/taui_arr[i]))
        G += Pi*log(Pi) + (1 - Pi)*log(1 - Pi) + log(2)
    end

    flag = 1
    for i in range(1, length(dwell_time_arr))

        Pi = 1 - 0.5*exp(-sqrt(dwell_time_arr[i]/taui_arr[i]))
        ln_diff=log(Pi)-log(1-Pi)
        dIiddi=(1/(2*sqrt(dwell_time_arr[i]*taui_arr[i])))*(1-Pi)*ln_diff
        d2Iiddi2=((1-Pi)/(4*dwell_time_arr[i]*taui_arr[i]))*
                    (1/Pi-ln_diff*(1+sqrt(taui_arr[i]/dwell_time_arr[i])))
        
        eigi=(dIiddi/G)^2-d2Iiddi2/G
        
        println("Eigenvalue is ", eigi)

        if eigi <= 10^(-8)

            flag = 0
            return flag

        end
    end

    return flag

end

function descent_dir(dwell_time_arr, taui_arr, alpha)
    # In this function, the descent direction for gradient descent is obtained.
    # If in the dwell time array, none of the dwell times are zero, then the negative
    # of the gradient is chosen as the descent direction. Else, if some of the dwell
    # times are zero, the sign of the corresponding coefficient in 
    # the gradient is checked. If it is positive, then the term is set to zero,
    # since we would then require to further reduce di below zero.

    # Obtaining the gradient
    grad = gradient_dwell_time_func(dwell_time_arr, taui_arr, alpha)
    # println("Gradient in the descent function is ", grad)

    # Checking if any of the dwell times is nearly zero
    for i in range(1, length(dwell_time_arr))
        # println("i is ", i)
        if dwell_time_arr[i] <= 0.000001
            if grad[i] > 0
                grad[i] = 0 # To ensure that we don't move in -grad[i] direction
                # for dwell_time_arr[i]
            end
        end
    end
    # println("Descent direction computed is ", grad)

    return grad

end

function nesterov_gradient_descent(taui_arr,alpha,eta;tolerance=.0001,initial=nothing)
    # eta is the learning rate
    if initial === nothing
        x_t = random_startpoint_generation(taui_arr)
    else
        x_t=initial
    end
    z_t=deepcopy(x_t)
    # println("Obtaining the descent direction")
    grad_t = -descent_dir(x_t, taui_arr, alpha)
    # iter = 1

    # time step 0
    # println("Descent direction is ", xdesc)
    lambda_t=0.

    while norm(grad_t) > tolerance # Performing gradient descent till norm condition
        # is satisfied

        # time step t
        # print(norm(grad_t),'\r')

        lambda_tplus1=(1+sqrt(1+4*lambda_t^2))/2
        gamma_t=(1-lambda_t)/lambda_tplus1

        z_tplus1=x_t+eta*grad_t

        x_tplus1=(1-gamma_t)*z_tplus1+gamma_t*z_t 
        

        # next step
        # clamps x onto R^+
        x_t=clamp.(x_tplus1,0.,Inf)
        
        # println("New point is ", x)
        grad_t = -descent_dir(x_t, taui_arr, alpha)

        lambda_t=lambda_tplus1
        z_t=z_tplus1
    end
    # println()
    return x_t

end

function simple_momentum_gradient_descent(taui_arr,alpha,eta,gamma,tolerance=.0001)
    # eta is the learning rate
    # gamma is the momentum parameter (larger gamma means less momentum)
    x=[]
    x = random_startpoint_generation(taui_arr)
    # println("Obtaining the descent direction")
    xdesc = -descent_dir(x, taui_arr, alpha)
    # iter = 1
    # println("Descent direction is ", xdesc)
    g=xdesc*gamma # initialize momentum as this

    while norm(xdesc) > tolerance # Performing gradient descent till norm condition
        # is satisfied
        # print(norm(xdesc),'\r')
        x+=eta*g

        # clamps x onto R^+
        x=clamp.(x,0.,Inf)

        # Updating xdesc
        # println("New point is ", x)
        xdesc = -descent_dir(x, taui_arr, alpha)
        #update momentum
        g=(1-gamma)*g+gamma*xdesc

    end
    return x

end

function gradient_descent(taui_arr, alpha, beta1, beta2)
    # Here, beta1 and beta2 are parameters in (0, 1) for the gradient descent using
    # backtracking. ini_dwell_time_arr is the initial starting point, taui_arr is
    # the array containing the tau values, alpha is the discount factor on
    # information gain.

    # Performing repeated gradient descents until the gradient is nearly zero
    # or the direction of descent is nearly zero
    flag = 0
    x = []
    while flag == 0

        # Obtaining the gradient at the initial point
        x = random_startpoint_generation(taui_arr)
        # println("Obtaining the descent direction")
        xdesc = -descent_dir(x, taui_arr, alpha)
        # iter = 1
        # println("Descent direction is ", xdesc)

        while norm(xdesc) > 0.0001 # Performing gradient descent till norm condition
            # is satisfied
            # print(norm(xdesc),'\r')

            # Perfoming a backtracking line search
            t = 0.1 # Initializing step size
            # println("Testing new point.")

            # Second condition checks if any of the y's become negative
            while (dwell_time_info_gain_func(x + t*xdesc, taui_arr, alpha) >
                dwell_time_info_gain_func(x, taui_arr, alpha)
                + beta1*t*(xdesc'gradient_dwell_time_func(x, taui_arr, alpha))) || any(y->y < 0, x + t*xdesc)

                # println("Shortening interval")
                t = beta2*t # Shortening the interval
                # iter += 1

            end

            # Updating x and xdesc
            x += t*xdesc
            # println("New point is ", x)
            xdesc = -descent_dir(x, taui_arr, alpha)

        end

        flag = 1

        # hessian test is unnecessary since we have a concave function

        # Obtaining the gradient
        #xgrad = gradient_dwell_time_func(x, taui_arr, alpha)
        #if norm(xgrad) <= 0.001 && all(y-> y >= 10^(-6), x)
        #    # Checking the Hessian if an internal point is obtained
        #    flag = hessian_test(x, taui_arr)
        #    # println("Testing Hessian at internal point ", x, ". flag value is ", flag)
        #end

    end

    return x

end

function random_startpoint_generation(taui_arr)
    # In this function, a random start point is generated for the gradient search.
    # Each target's dwell time is selected randomly between 0 (excluding) and
    # 3*taui_arr[i]

    ini_dwell_time = []
    for i in range(1, length(taui_arr))
        append!(ini_dwell_time, 0)#3*rand()*taui_arr[i])
    end

    return ini_dwell_time
    
end