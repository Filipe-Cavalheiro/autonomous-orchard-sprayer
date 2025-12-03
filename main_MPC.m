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

ref_pos = [Tree_pos(:,1)+Param.x_spacing/2,Tree_pos(:,2)];

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
ref_pos = [ref_pos(:,1), zeros(size(ref_pos,1), 1), ref_pos(:,2), zeros(size(ref_pos,1), 1)];
%ref_pos = [[ref_pos(1,1), ref_pos(1,2)-Param.y_spacing]; ref_pos; [ref_pos(end,1), ref_pos(end,2)+Param.y_spacing]];
mpc_ref_pos = reshape(ref_pos.', [], 1);
tractor_pos = zeros(nx, 1);

state = tractor_pos;
state_history = state;   % Store for plotting
k = 1;   % reference index
y_ref = mpc_ref_pos(k:k+3);
Yref = repmat(y_ref,N+1,1);
Yref = Yref(1:(N+1)*ny);
plot(ref_pos(:,1),ref_pos(:,3), 'HandleVisibility','on') 
while k < size(ref_pos,1)*2-3 %run until last reference
    u = controllerSimple(K, Ky, state, Yref, nu);
    state = A_mpc * state + B_mpc * u;
    % Ref updating
    if (Yref(5)-state(1))^2 + (Yref(7)-state(3))^2 < 0.2
        k = k+4; %since the ref is stacked in a row its +4
        y_ref = mpc_ref_pos(k:k+3);
        Yref = repmat(y_ref,N+1,1);
        Yref = Yref(1:(N+1)*ny);
    end
    state_history = [state_history, state];
    if size(state_history, 2) > 100000
        disp("system took too long :(")
        break;
    end
end

animate = 0;
plot_figures(animate, Tree_pos,ref_pos, state_history, linkage_dim)