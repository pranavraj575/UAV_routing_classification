# Implementation of gradient descent on the objective function
# implemented classic gradient descent as well as descent with momentum
using LinearAlgebra, Random


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
    # println("Descent direction is ", grad_t)
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
    return x_t

end

function simple_momentum_gradient_descent(taui_arr,alpha,eta,gamma;tolerance=.0001)
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
        append!(ini_dwell_time, 0)# 3*rand()*taui_arr[i])
    end

    return ini_dwell_time
    
end