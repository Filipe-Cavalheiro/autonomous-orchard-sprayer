function u = controllerSimple(K, Ky, state, Yref, nu)
    %CONTROLLERSIMPLE Summary of this function goes here
    %   Detailed explanation goes here
    
    Uvec = -K * state + Ky * Yref;   % full horizon control sequence
    u = Uvec(1:nu);                  % apply only first control
end
