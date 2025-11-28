clear; clc; close all;

% Simulation parameters
Tend = 60;
dTi = 0.01;  % inner-loop and simulation sampling period
Nsim = round(Tend/dTi)+1;
result = [];

state = [0 0 0 0]';
k = 1;   % reference index

state_history = state;   % Store for plotting
actuation_history = [];

% design parameters 
alpha = 1;    % back-stepping tuning (>0)
beta  = 1;    % back-stepping tuning (>0)

A = [1  dTi  0  0;
     0  1   0  0;
     0  0   1  dTi;
     0  0   0  1];

B = [0.5*dTi^2  0;
     dTi        0;
     0     0.5*dTi^2;
     0     dTi];

% Lyapunov Function 
V = @(x) 0.5*x'*x;
LfV = @(x) x'*A*x;
LgV = @(x) x'*B;

Yref = [10 0 10 0]';

% simulate
for i= 0:Nsim
    u = [0; 0];

    % --- compute error coordinates (state minus reference) ---
    error = state - Yref;   % error state [e1; e2; e3; e4]

    if LfV(error) + V(error) > 10^-5 % zero might be a bad idea since the math might not stabilize 
        % virtual control phi (for x2)
        omega1 = -alpha * error(1); % omega(x) = -(a+alpha)*x1  (error form)
        z1   = error(2) - omega1;   % z = x2 - omega  (error coordinates)
        
        % backstepping control v (for z dynamics) -> this will be u1
        v1 = - error(1) - (alpha + beta) * z1 + alpha * alpha * error(1);
        
        omega2 = -alpha * error(3);
        z2   = error(4) - omega2;
        
        v2 = - error(3) - (alpha + beta) * z2 + alpha * alpha * error(3);
        
        u = [v1; v2];
    end
    % max velocity of 40 Km/h
    u = min(u, [40; 40]);

    state = A * state + B * u;     % continuous-time derivative
    state_history = [state_history, state];

end

plot(state_history(1,:), state_history(3,:), 'r-', 'LineWidth', 1.5);