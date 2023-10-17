syms d1 d2 d alpha tau
Pi = @(d, tau) 1 - 0.5.*exp(-d./tau);
f = @(d1, d2, alpha, tau) exp(-alpha.*(d1 + d2)).*...
    (Pi(d1, tau).*log(Pi(d1, tau)) + (1 - Pi(d1, tau)).*log(1 - Pi(d1, tau)) + log(2)...
     + Pi(d2, tau).*log(Pi(d2, tau)) + (1 - Pi(d2, tau)).*log(1 - Pi(d2, tau)) + log(2));

tauval = 1;
alphaval = 0.55;
d1val = linspace(0, 4, 100);
d2val = linspace(0, 4, 100);
[X, Y] = meshgrid(d1val, d2val);
Z = f(X, Y, alphaval, tauval);
contour(X, Y, Z, 30, 'ShowText', 'on')