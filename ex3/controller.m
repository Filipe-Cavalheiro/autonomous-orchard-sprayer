function u = controllerLine(state, Yref, v_ref)
    v = v_ref;
    
    max_unorm = 2*pi;
    g0 = @(z) [cos(z); sin(z)];
    g0dot = @(z) [-sin(z); cos(z)];
    B = [0; 0; 1];

    % Define V
    beta = 1; % Gain to control how attracted to the ref bot is (the bigger the better)
    alpha = 1; % Gain to control how strongly bot corrects to ref
    miuV = 1;
    
    phi = @(x1,Yref) -alpha*(x1-Yref); % Virtual controller for V error
    phidot = @(x2,v) -alpha*v*g0(x2);
    
    V1 = @(x1,Yref) 1/2*norm(x1-Yref)^2;
    V1dot = @(x1,x2,v,Yref) (x1-Yref)'*v*g0(x2);
    
    V = @(x1,x2,v,Yref) V1(x1,Yref) + (1/(2*miuV))*norm(v*g0(x2)-phi(x1,Yref))^2;
    
    % Calculate closed form expression %
    kappa = cell(4,2); % = [ustar; condition]
    
    % CLF
    kappa{1,2} = @(x1,x2,e,v,Yref) (-beta*V(x1,x2,v,Yref)+e'*phidot(x2,v)-V1dot(x1,x2,v,Yref)/(v*e'*g0dot(x2))^2)*(v*e'*g0dot(x2));
    kappa{2,2} = @(x1,x2,e,v,Yref) -(-beta*V(x1,x2,v,Yref)+e'*phidot(x2,v)-V1dot(x1,x2,v,Yref)) > 0 && (v*e'*g0dot(x2))^2 ~= 0;
    
    kappa{1,4} = zeros(size(B,2));
    kappa{2,4} = 0;


    x1sim = state(1:2);
    x2sim = state(3);
    e1sim = v*g0(x2sim) - phi(x1sim,Yref);

    if kappa{2,2} (x1sim,x2sim,e1sim,v,Yref)
        u = kappa{1,2} (x1sim,x2sim,e1sim,v,Yref);
    else
        u = kappa{1,4};
    end

    if norm(u) > max_unorm
        u = max_unorm*u/norm(u);
    end
end
