% Simulation parameters
Param.Tend = 60;
Param.dTi = 0.01;  % inner-loop and simulation sampling period
Param.Nsim = round(Param.Tend/Param.dTi)+1;

% Ambient parameters
Param.x_spacing = 6;   % spacing between trees in x-direction [m]
Param.y_spacing = 4;   % spacing between trees in y-direction [m]
Param.col_length = 20; % length of rows (:= column of trees) [m]
Param.Nx = Param.col_length/Param.y_spacing; % number of trees in x direction
Param.Ny = 4;         % number of rows in y direction

% System parameters
linkage_dim = 3; % Trailer length

%   TRAILER DYNAMICS
%       ẋ = v * cos(φ − θ) * cos(θ)
%       ẏ = v * cos(φ − θ) * sin(θ)
%       φ̇ = ω
%       θ̇ = (v / l) * sin(φ − θ)

g0T = @(zeta,v) v.*[cos(zeta(1)-zeta(2))*cos(zeta(2));
                    cos(zeta(1)-zeta(2))*sin(zeta(2))];
g1T = @(zeta,v) v/linkage_dim*sin(zeta(1)-zeta(2));

gtheta = @(zeta,v) v.*[sin(zeta(1)-zeta(2))*cos(zeta(2))-cos(zeta(1)-zeta(2))*sin(zeta(2));
                       sin(zeta(1)-zeta(2))*sin(zeta(2))+cos(zeta(1)-zeta(2))*cos(zeta(1))];
gu = @(zeta,v) -v.*[sin(zeta(1)-zeta(2))*cos(zeta(2));
                   sin(zeta(1)-zeta(2))*sin(zeta(2))];

Atrailer = @(zeta,v) [g0T(zeta,v);
                     0;
                     g1T(zeta,v)];
Btrailer = [0;0;1;0];
Ctrailer = eye(4);

%   TRACTOR DYNAMICS
% xdot = v*cos(z)   =>  xdot = v*g0(z)
% ydot = v*sin(z)       zdot = u
% zdot = u

g0 = @(z) [cos(z); sin(z)];

% dg0/dt = u*g0dot
g0dot = @(z) [-sin(z); cos(z)];
%g0Tdot = @(x,z) [-(u-1/linkage_dim*sin(z-x))*sin(z-x)*cos(x) - 1/linkage_dim*sin(z-x)*cos(z-x)*sin(x);];

A = @(x,v) [v*g0(x(end)); 0];
B = [0; 0; 1];
C = eye(3);

nx = size(B, 1);
nu = size(B, 2);
ny = size(C, 1);

max_unorm = 2*pi;
init_tractor_pos = zeros(nx, 1);
init_trailer_pos = [-linkage_dim;0;0.1;0];

% ===v LYAPUNOV/BARRIER FUNCTIONS v=== %
radius = 1;
center = [1;1];

% Define V for Tractor
beta = 1; % Gain to control how attracted to the ref bot is (the bigger the better)
alpha = 1; % Gain to control how strongly bot corrects to ref
miuV = 1;

phiT = @(x,Yref) -alpha*(x-Yref); % Virtual controller for V error
phidotT = @(zeta,v) -alpha*g0T(zeta,v);

V1T = @(x,Yref) 1/2*norm(x-Yref)^2;
V1dotT = @(x,zeta,v,Yref) (x-Yref)'*g0T(zeta,v);

VT = @(x,zeta,v,Yref) V1T(x,Yref) + (1/(2*miuV))*norm(g0T(zeta,v)-phiT(x,Yref))^2;
VdotT = @(x,zeta,u,v,Yref) V1dotT(x,zeta,v,Yref) + (1/miuV)*(g0T(x,zeta,v)-phiT(x,Yref))'*(u.*gu(zeta,v)+g1T(zeta,v).*gtheta(zeta,v)-phidotT(zeta,v));

% phi = @(x1,Yref) -alpha*(x1-Yref); % Virtual controller for V error
% phidot = @(x2,v) -alpha*v*g0(x2);
% 
% V1 = @(x1,Yref) 1/2*norm(x1-Yref)^2;
% V1dot = @(x1,x2,v,Yref) (x1-Yref)'*v*g0(x2);
% 
% V = @(x1,x2,v,Yref) V1(x1,Yref) + (1/(2*miuV))*norm(v*g0(x2)-phi(x1,Yref))^2;
% Vdot = @(x1,x2,u,v,Yref) V1dot(x1,x2,v) + (1/miuV)*(v*g0(x2)-phi(x1,Yref))'*(v*u*g0dot(x2)-phidot(x2,v));

% Define h
sigma = 1; % Gain to control how "scared" bot is of obstacle (the smaller the better)
omega = 1; % Gain to control how strongly bot overcorrects
miuh = 1;

psi = @(x1,center) omega*(x1-center); % Virtual controller for h error
psidot = @(x2,v) omega*v*g0(x2);

h1 = @(x1,center,radius) norm(x1-center)^2 - radius^2;
h1dot = @(x1,x2,v,center) 2*(x1-center)'*v*g0(x2);

h = @(x1,x2,v,center,radius) h1(x1center,radius) + 1/(2*miuh)*norm(v*g0(x2)-psi(x1,center))^2;
hdot = @(x1,x2,u,v,center,radius) h1dot(x1,x2,v,center) + 1/miuh*(v*g0(x2)-psi(x1,center))'*(v*u*g0dot(x2)-psidot(x2,v));

% Calculate closed form expression %
kappa = cell(4,2); % = [ustar; condition]
kappaT = cell(4,2); % = [ustar; condition]

% % CLF+CBF
% kappa{1,1} = @(x1,x2,eV,eh,Yref) (-beta*V(x1,x2,Yref)+eV'*phidot(x2)-V1dot(x1,x2,Yref)...
%     -sigma*h(x1,x2)+eh'*psidot(x2)-h1dot(x1,x2))/(norm(eV+eh)^2)*(eV+eh);
% kappa{2,1} = @(x1,x2,eV,eh,Yref) -(-beta*V(x1,x2,Yref)+eV'*phidot(x2)-V1dot(x1,x2,Yref)) > 0 && ...
%     -(-sigma*h(x1,x2)+eh'*psidot(x2)-h1dot(x1,x2)) < 0 && norm(eV+eh)^2 ~= 0;

% CLF
% kappa{1,2} = @(x1,x2,e,v,Yref) (-beta*V(x1,x2,v,Yref)+e'*phidot(x2,v)-V1dot(x1,x2,v,Yref)/(v*e'*g0dot(x2))^2)*(v*e'*g0dot(x2));
% kappa{2,2} = @(x1,x2,e,v,Yref) -(-beta*V(x1,x2,v,Yref)+e'*phidot(x2,v)-V1dot(x1,x2,v,Yref)) > 0 && (v*e'*g0dot(x2))^2 ~= 0;
kappa{1,2} = @(x,zeta,e,v,Yref) -(e'*gu(zeta,v)*(V1dotT(x,zeta,v,Yref)+beta*VT(x,zeta,v,Yref)-e'*(phidotT(zeta,v)-g1T(zeta,v).*gtheta(zeta,v))))/(((e'*gu(zeta,v)).*e)'*gu(zeta,v));
kappa{2,2} = @(x,zeta,e,v,Yref) -(V1dotT(x,zeta,v,Yref)+beta*VT(x,zeta,v,Yref)-e'*(phidotT(zeta,v)-g1T(zeta,v).*gtheta(zeta,v))) > 0 && (((e'*gu(zeta,v)).*e)'*gu(zeta,v)) ~= 0;

% CBF
kappa{1,3} = @(x1,x2,e) (-sigma*h(x1,x2)+e'*psidot(x2)-h1dot(x1,x2)/norm(e)^2)*e;
kappa{2,3} = @(x1,x2,e) -(-sigma*h(x1,x2)+e'*psidot(x2)-h1dot(x1,x2)) < 0 && norm(e)^2 ~= 0;

kappa{1,4} = zeros(size(B,2));
kappa{2,4} = 0;

% ===v SPRAYER v=== %
% sIn := Spray Input (varies between 0 and 1)
%spray_radius =  @(sIn) 1.5 + sIn*2.5;
spray_radius = 1.5;
spray_Lms = 0.15/(0.5*pi*spray_radius^2); % [L/s/m^2]

% Angles
thetaTr  = linspace(pi/2, 3*pi/2, 100);     % semicircle

% Shifted sprayer semicircle
sprayerPoints = spray_radius*[cos(thetaTr); sin(thetaTr)]';

pSprayinit = polyshape(sprayerPoints);
%pSpray = translate(pSpray,init_trailer_pos(1:2)');