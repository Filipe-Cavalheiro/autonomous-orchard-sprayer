clear; clc; close all;

state = [0 0 0 0]';
state_history = state;   % Store for plotting
actuation_history = [];

% design parameters 
alpha = 1;    % back-stepping tuning (>0)
beta  = 1;    % back-stepping tuning (>0)
linkage_dim = 3;

% Lyapunov Function 
V = @(x) 0.5*x'*x;
LfV = @(x) 0;
Lg1V = @(x) x'*[ cos(x(3)-x(4))*cos(x(4));
             cos(x(3)-x(4))*sin(x(4));
             0                      ;
             (1/linkage_dim)*sin(x(3)-x(4))];

Lg2V = @(x) x'*[0;
                0;
                1 ;
                0];

Yref = [10 10 0 0]';

A = zeros(4,4);   % drift vector field is zero

B = @(x) [ cos(x(3)-x(4))*cos(x(4)),   0 ;
             cos(x(3)-x(4))*sin(x(4)),   0 ;
             0,                         1 ;
             (1/linkage_dim)*sin(x(3)-x(4)), 0 ];

% simulate
for i = 0:10000
    u = [0; 0];
    
    % ang = atan2(Yref(2)-state(2), Yref(1)-state(1));
    % Yref(3) = ang;
    % Yref(4) = ang;

    error = state - Yref;   % error state [e1; e2; e3; e4]
    if LfV(error) + V(error) > 10^-5
        u1 = -V(error)/(Lg1V(error)^2+Lg2V(error)^2) *Lg1V(error)';
        u2 = -V(error)/(Lg1V(error)^2+Lg2V(error)^2) *Lg2V(error)';
        u = [u1;u2];
    end

    u = max(min(u, [40; 40]), [-40; -40]);

    dt = 0.001;
    state = state + dt * (B(state) * u);
    state_history = [state_history, state];
end

plot(state_history(1,:), state_history(3,:), 'r-', 'LineWidth', 1.5);