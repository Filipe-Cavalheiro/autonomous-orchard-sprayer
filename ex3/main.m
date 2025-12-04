% =========================================================================
% Project: Cyber-Physical Control Systems – Project Part I (2025–2026)
% =========================================================================
%
% Description:
%   This project focuses on modeling and controlling an agricultural tractor
%   used for spraying olive tree plantations. The tractor dynamics are 
%   governed by a nonlinear kinematic model defined by:
%
%       ẋ = v * cos(φ − θ) * cos(θ)
%       ẏ = v * cos(φ − θ) * sin(θ)
%       φ̇ = ω
%       θ̇ = (v / l) * sin(φ − θ)
%
%   where:
%       x, y  – positional coordinates of point P (tractor reference point)
%       φ     – steering angle of the front wheels
%       θ     – heading angle of the tractor
%       v     – linear velocity
%       ω     – angular velocity of the steering
%       l     – wheelbase (3 meters)
%
%   The agricultural setup considers olive tree rows spaced 6 m apart with
%   4 m inline spacing. The sprayer can cover an adjustable range between 
%   1.5 m and 4 m, controlled by a variable z ∈ [0, 1]. Consumption of the 
%   spray solution varies linearly between 9 and 14.4 liters/min depending 
%   on the selected range.
%
%   The main goals of the project are:
%     1. Implement a controller to drive the tractor to a reference position.
%     2. Develop a line-following controller robust to terrain disturbances.
%     3. Design a full controller for automatic spray operation minimizing
%        cost per hectare, while meeting product and safety constraints.
%
%   This code implements the main simulation and control logic for the 
%   Cyber-Physical Control Systems course (CPCS), focusing on control design,
%   optimization, and performance evaluation.
%
% -------------------------------------------------------------------------
% Authors:
%   Name 1: Filipe Cavalheiro
%   Name 2: António Canteiro
%   Name 3: Rafael Louro
%
% -------------------------------------------------------------------------
% License:
%
%   This program is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with this program.  If not, see <https://www.gnu.org/licenses/>.
%
% =========================================================================
clear; clc; close all;

init_vars;

% Generate grid of tree positions
[x, y] = meshgrid(4:Param.x_spacing:4+(Param.Nx-1)*Param.x_spacing, 4:Param.y_spacing:4+(Param.Ny-1)*Param.y_spacing);
Tree_pos = [x(:), y(:)];

Tree_poly = cell(size(Tree_pos,1),1);
theta2 = linspace(0, 2*pi, 200);    % full circle
tree_radius = 2;
tree = [tree_radius*cos(theta2); tree_radius*sin(theta2)];
for ind = 1:size(Tree_pos,1)
    % TREE as polyshape 
    Tree_poly{ind} = polyshape(tree(1,:)+Tree_pos(ind,1), tree(2,:)+Tree_pos(ind,2));
end

ref_pos = [Tree_pos(:,1)+Param.x_spacing/2,Tree_pos(:,2)];
%ref_pos = ref_pos(1:Param.Ny:size(Tree_pos,1),:);

% Find unique groups
groups = unique(ref_pos(:,1));

result = [];

for k = 1:length(groups)
    g = groups(k);

    % extract subset
    subset = ref_pos(ref_pos(:,1) == g, :);
    subset = [subset; [subset(end, 1), subset(end, 2)+Param.y_spacing]];
    subset = [[subset(1, 1), subset(1, 2)-Param.y_spacing]; subset];

    if mod(k,2) == 0
        subset = flipud(subset);
    end

    result = [result; subset];
end
ref_pos = result;
ref_pos = [ref_pos(:,1), ref_pos(:,2)];
%ref_pos = [[ref_pos(1,1), ref_pos(1,2)-Param.y_spacing]; ref_pos; [ref_pos(end,1), ref_pos(end,2)+Param.y_spacing]];
mpc_ref_pos = reshape(ref_pos.', [], 1);

state = init_tractor_pos;
state_trailer = init_trailer_pos;
k = 1;   % reference index
%Yref = mpc_ref_pos(k:k+3);
Yref = ref_pos(k,:)';

state_history = state;   % Store for plotting
state_tr_history = state_trailer;
actuation_history = [];

spray_ammount = zeros(size(Tree_poly));

% % design parameters 
% alpha = 1;    % back-stepping tuning (>0)
% beta  = 1;    % back-stepping tuning (>0)
% 
% % Lyapunov Function 
% V = @(x) 0.5*x'*x;
% LfV = @(x) x'*A*x;
% LgV = @(x) x'*B;

% simulate
v = 40/3.6;
time = 0;
while k < size(ref_pos,1)
    disp("current ref:" + k);
    % time = time+1;
    % if (time > 5000)
    %     break;
    % end
    % u = [0; 0];

    if norm(Yref-state(1:2))^2 < 0.2
        k = k+1; %since the ref is stacked in a row its +4
        Yref = ref_pos(k,:)';
    end

    u = controller(state, Yref, 10);

    [t,y] = ode45(@(t,y) odeProj1(t,y,u,A(state,v),B,C), [0 Param.dTi], state);
    state = y(end,:)';

    [t,y] = ode45(@(t,y) odeProj1(t,y,u,Atrailer(state_trailer,v),Btrailer,Ctrailer), [0 Param.dTi], state_trailer);
    state_trailer = y(end,:)';

    % Translate and Rotate Spray Area
    pSpray = translate(pSprayinit,state_trailer(1:2)');
    if state_trailer(end) ~= 0
        pSpray = rotate(pSpray, state_trailer(end));
    end

    for ind = 1:size(Tree_poly,1)
        pAux = intersect(pSpray, Tree_poly{ind});
        if pAux.NumRegions ~= 0
            % Intersection area
            spray_ammount(ind) = spray_ammount(ind)+area(pAux)*spray_Lms*Param.dTi;
        end
    end

    state_history = [state_history, state];
    state_tr_history = [state_tr_history,state_trailer];
    actuation_history = [actuation_history, u];
end

animate = 1;
plot_figures(animate, Tree_pos,ref_pos, state_tr_history, state_history, actuation_history, pSprayinit,linkage_dim)