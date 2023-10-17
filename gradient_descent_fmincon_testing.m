alpha = 2;
tau = 0.7;

fun = @objfungrad;
nonlcon = @confungrad;

function f = objfungrad(x)
    
    P1 = 1 - 0.5*exp(-x(1)/tau);
    P2 = 1 - 0.5*exp(-x(2)/tau);
    f = alpha*(x(1) + x(2)) - log(P1*log(P1) + (1 - P1)*log(1 - P1) + 2*log(2)...
        + P2*log(P2) + (1 - P2)*log(1 - P2));
    
end

function [c,ceq] = confungrad(x)
    c(1) = - x(1); % Inequality constraints
    c(2) = - x(2);
    c(3) = -x(1)*x(2) + 0.00001; % Ensuring at most one of x1 and x2 can be zero
    % No nonlinear equality constraints
    ceq=[];
end