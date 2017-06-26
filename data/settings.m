function [param] = settings()

% System dynamics parameters 



param.t0 = 0; % initial time
param.tf = 20; % final time %27

% Nominal system input
param.u_nominal = @(x) 0; % if u is a function of x


param.ulen = 2; % number of inputs

%Numerical tolerance
param.epsilon = 1e-5;

end

