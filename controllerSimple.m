function u = controllerSimple(K, Ky, state, Yref, nu)
    %CONTROLLERSIMPLE Summary of this function goes here
    %   Detailed explanation goes here
    % Homework 2: 1(b) -- DC motor MPC (horizon N = 10) tracking square ref 0 <-> 10 deg/s
    % Motor parameters (given)
    
    % compute first control action using the analytic unconstrained solution
    Uvec = -K * state + Ky * Yref;   % full horizon control sequence
    u = Uvec(1:nu);                  % apply only first control
end
