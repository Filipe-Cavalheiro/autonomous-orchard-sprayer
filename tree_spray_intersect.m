close all;
clearvars;
clc;

% sprayer 180° range: 1.5 m → 4 m
% spray_input z ∈ [0,1] controls range AND spray consumption
% 0 → 0.15 L/s, 1 → 0.24 L/s (linear)

Tend = 2;
dTi = 0.01;  % simulation sampling period
Nsim = round(Tend/dTi)+1;

% Parameters
car_speed_kph = 40; % [tractor speed in Km/h]
car_speed_mps = car_speed_kph/3.6; % [tractor speed in m/s]
tree_radius = 2;
spray_input = 0; % varies between 0 and 1
spray_radius = 1.5 + spray_input*2.5;

% ---- Spray flow rate and spray area ----
spray_rate = 0.15 + spray_input*(0.24 - 0.15);    % L/s
spray_area_total = 0.5 * pi * spray_radius^2;     % semicircle area

% Angles
theta  = linspace(pi/2, 3*pi/2, 100);     % semicircle
theta2 = linspace(0, 2*pi, 200);    % full circle

theta  = theta(1:end-1);
theta2 = theta2(1:end-1);

% TREE as polyshape
tree = [tree_radius*cos(theta2); tree_radius*sin(theta2)];
pTree = polyshape(tree(1,:), tree(2,:));

% Visualization
figure; hold on; axis equal;
plot(pTree, 'FaceColor',[0 1 0], 'FaceAlpha',0.4);
title('Intersection area');

% Simulation variables
initial_tractor_pos = [0, -2];
shift = zeros(Nsim, 1);
A = zeros(Nsim, 1);                 % intersection area per step
total_area = zeros(Nsim, 1);        % cumulative intersection area
product_step = zeros(Nsim, 1);      % product applied per time step
product_total = zeros(Nsim, 1);     % cumulative product applied

% Sprayer semicircle
xSprayer = initial_tractor_pos(1) + spray_radius*cos(theta) - tree_radius;
ySprayer = initial_tractor_pos(2) + spray_radius*sin(theta);

pSprayer = polyshape(xSprayer, ySprayer);

% ---------------- SIMULATION LOOP -----------------
for i = 1:Nsim
    
    shift(i) = dTi*car_speed_mps*i;

    pSprayer = translate(pSprayer,[0,shift(i)]);

    pIntersect = intersect(pSprayer, pTree);

    % Intersection area
    A(i) = area(pIntersect);

    % Cumulative area
    if i == 1
        total_area(i) = A(i);
    else
        total_area(i) = total_area(i-1) + A(i);
    end

    % ---------------- PRODUCT DEPOSITION ------------------
    % product proportional to intersection fraction
    product_step(i) = spray_rate * dTi * (A(i) / spray_area_total);

    if i == 1
        product_total(i) = product_step(i);
    else
        product_total(i) = product_total(i-1) + product_step(i);
    end
    % ------------------------------------------------------

    disp(['dt = ', num2str(dTi), ...
          ' → Area = ', num2str(A(i)), ...
          ' → Product applied (L) = ', num2str(product_step(i))]);

    % Animation plot
    clf; hold on; axis equal;
    plot(pTree,    'FaceColor',[0 1 0],   'FaceAlpha',0.4);
    plot(pSprayer, 'FaceColor',[0 0.5 1], 'FaceAlpha',0.4);
    plot(pIntersect,'FaceColor',[1 0 0],  'FaceAlpha',0.6);
    title(['Intersection area = ', num2str(A(i))]);
    drawnow;
    pause(0.01);
end

% ---- Final cumulative area plot ----
figure;
plot(shift, total_area, 'LineWidth', 2);
xlabel('Vertical shift');
ylabel('Cumulative intersection area');
title('Cumulative Area vs Shift');
grid on;

% ---- Step area plot ----
figure;
plot(shift, A, 'LineWidth', 2);
xlabel('Vertical shift');
ylabel('Step intersection area');
title('Step Area vs Shift');
grid on;

% ---- Product applied over time ----
figure;
plot(shift, product_total, 'LineWidth', 2);
xlabel('Vertical shift');
ylabel('Cumulative product applied [L]');
title('Total Product Deposited on Tree');
grid on;

figure;
plot(shift, product_step, 'LineWidth', 2);
xlabel('Vertical shift');
ylabel('Product deposited per step [L]');
title('Step Product Deposition');
grid on;
