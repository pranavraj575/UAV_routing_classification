syms d d1 d2 alpha tau tau1 tau2

tau1_val = 0.7; tau2_val = 0.7; alpha_val = 2;

Pi = @(d, tau) 1 - 0.5.*exp(-d./tau);
f = @(d1, d2, alpha, tau1, tau2) exp(-alpha.*(d1 + d2)).*...
    (Pi(d1, tau1).*log(Pi(d1, tau1)) + (1 - Pi(d1, tau1)).*log(1 - Pi(d1, tau1))...
    + Pi(d2, tau2).*log(Pi(d2, tau2)) + (1 - Pi(d2, tau2)).*log(1 - Pi(d2, tau2))...
    + 2*log(2));
d1_lin = linspace(0.00001, 2, 100);
d2_lin = linspace(0.00001, 2, 100);
[X, Y] = meshgrid(d1_lin, d2_lin);
Z = f(X, Y, alpha_val, tau1_val, tau2_val);
contour(X, Y, Z, 20, 'ShowText', 'on')
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 13)
xlabel('$d_1$', 'Interpreter', 'latex', 'FontSize', 15)
ylabel('$d_2$', 'Interpreter', 'latex', 'FontSize', 15)
title('Information gain for $\alpha = 2$, $\tau_1 = \tau_2 = 0.7$', 'Interpreter', 'latex',...
    'FontSize', 15)