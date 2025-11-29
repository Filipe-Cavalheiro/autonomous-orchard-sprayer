close all;
clearvars;
clc;

% Parameters
r = 2;
theta  = linspace(0, -pi, 100);     % semicircle
theta2 = linspace(0, 2*pi, 200);    % full circle

% Remove duplicate end points
theta = theta(1:end-1);
theta2 = theta2(1:end-1);

% Generate TREE as a circle centered at (0,0)
tree = [r*cos(theta2); r*sin(theta2)];
pTree = polyshape(tree(1,:), tree(2,:));

% Visualization setup
figure; hold on; axis equal;
plot(pTree, 'FaceColor',[0 1 0], 'FaceAlpha',0.4);
title('Intersection area');

% Shifts
shifts = -r:0.1:2*r;
t_sim = numel(shifts);
total_area = zeros(t_sim, 1);

k = 1;
for shift = shifts

    % Shifted SEMICIRCLE (sprayer)
    xSprayer = r*cos(theta) - r;      
    ySprayer = shift + r*sin(theta);

    % Build polyshape (force clean boundary)
    pSprayer = polyshape(xSprayer, ySprayer);
    
    % Intersection polygon
    pIntersect = intersect(pSprayer, pTree);

    % Area of intersection
    A(k) = area(pIntersect);

    % Safe cumulative sum
    if k == 1
        total_area(k) = A(k);
    else
        total_area(k) = total_area(k-1) + A(k);
    end

    disp(['shift = ', num2str(shift), ' → Intersection area = ', num2str(A(k))]);

    % Plot animation
    clf; hold on; axis equal;
    plot(pTree,    'FaceColor',[0 1 0],   'FaceAlpha',0.4);
    plot(pSprayer, 'FaceColor',[0 0.5 1], 'FaceAlpha',0.4);
    plot(pIntersect,'FaceColor',[1 0 0],  'FaceAlpha',0.6);
    title(['Intersection area = ', num2str(A(k))]);
    drawnow;
    pause(0.05);

    k = k+1;
end

% Final cumulative area plot
figure;
plot(shifts, total_area, 'LineWidth', 2);
xlabel('Vertical shift');
ylabel('Cumulative intersection area');
title('Cumulative Area vs Shift');
grid on;

% Final cumulative area plot
figure;
plot(shifts, A, 'LineWidth', 2);
xlabel('Vertical shift');
ylabel('Step intersection area');
title('Step Area vs Shift');
grid on;