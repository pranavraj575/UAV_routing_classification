# testing gradient descent functions

using Random, Plots, LaTeXStrings#; pythonplot()
include(joinpath("..","src","grad_descent.jl"))

function testing_gradient_descent(taui_arr = [], alpha = NaN, figure_name = joinpath("output", "test.pdf"))
    if !isdir("output")
        mkdir("output")
    end

    # Generating random tau values if empty
    if isempty(taui_arr)
        taui_arr = [rand()*2, rand()*2]
    end
    # Generating a random alpha value if not given
    if isnan(alpha)
        alpha = rand()
    end

    println("tau and alpha are ", taui_arr, alpha)

    # Running gradient descent
    x = nesterov_gradient_descent(taui_arr, alpha,.1)
    println("Solution obtained is ", x)
    # Plotting the function and the solution from gradient descent
    plotting_dwell_time_on_F(alpha, taui_arr, x, figure_name)
end

function plotting_dwell_time_on_F(alpha, taui_arr, x, figure_name)

    d1 = range(0, 2*x[1], length = 100)
    d2 = range(0, 2*x[2], length = 100)
    z = @. dwell_time_info_gain_func_two_dwell_times(d1', d2, taui_arr[1], taui_arr[2], alpha)
    contour(d1, d2, z, levels = 20, color=:turbo, clabels=true, cbar=true, lw=3,
    tickfontsize = 20, colorbar_tickfontsize = 20)
    # surface(d1, d2, z)
    xlabel!(L"d_1", guidefontsize = 25)
    ylabel!(L"d_2", guidefontsize = 25)
    scatter!([x[1]], [x[2]], color = "red", markersize = 10, legend = false)
    xlims!(0, last(d1))
    ylims!(0, last(d2))
    savefig(figure_name)
    # return d1, d2, z, x

end

function dwell_time_info_gain_func_two_dwell_times(d1, d2, tau1, tau2, alpha)
    # In this function, the negative log of the information gain function value
    # is obtained given the two dwell times, the tau array, and the alpha value.
    
    # Computing the "G" function
    # println("Computing P1 and P2")
    P1 = 1 - 0.5*exp(-sqrt(d1/tau1))
    P2 = 1 - 0.5*exp(-sqrt(d2/tau2))
    G = P1*log(P1) + (1 - P1)*log(1 - P1) + P2*log(P2) + (1 - P2)*log(1 - P2) + 2*log(2)
    
    # return alpha*(d1 + d2) - log(G)
    return exp(-alpha*(d1 + d2))*G
    
end

function plot_info_gain(rng,tau,figure_name)
    plot()
    res=[]
    for d in rng
        P = 1 - 0.5*exp(-sqrt(d/tau))
        Ig=P*log(P)+(1-P)*log(1-P)+log(2)
        res=push!(res,Ig)
    end
    plot(rng,res,legend=false)
    xlabel!("dwell time", guidefontsize = 10)
    ylabel!("information gain", guidefontsize = 10)
    savefig(figure_name)
end

function plot_objective_wrt_d(rng,alpha,k,figure_name=joinpath("output","obj_plot.pdf"),dim=2)
    plot()
    res=[]
    taui=[k for i in range(1,dim)]
    for d in rng
        neg_log=dwell_time_info_gain_func([d for i in range(1,dim)],taui,alpha)
        value=(-neg_log)

        res=push!(res,value)
    end
    plot(rng,res,legend=false)
    xlabel!("dwell time", guidefontsize = 10)
    ylabel!("objective function", guidefontsize = 10)

    savefig(figure_name)
end

function plot_dwell_time_wrt_k(rng,alpha,figure_name=joinpath("output", "dweel_plot.pdf"),dim=2)
    plot()
    res=[]
    for k in rng
        taui=[k for i in range(1,dim)]
        d_vec=gradient_descent(taui,alpha,.2,.5)

        res=push!(res,d_vec[1])
    end
    plot(rng,res,legend=false)
    xlabel!("k", guidefontsize = 10)
    ylabel!("dwell time", guidefontsize = 10)
    
    savefig(figure_name)
end
