% Simulation parameters
Param.Tend = 60;
Param.dTi = 0.01;  % inner-loop and simulation sampling period
Param.Nsim = round(Param.Tend/Param.dTi)+1;
Param.g = 9.81;     % earth gravity

% Ambient parameters
Param.x_spacing = 6;   % spacing between trees in x-direction [m]
Param.y_spacing = 4;   % spacing between trees in y-direction [m]
Param.col_length = 100; % length of rows (:= column of trees) [m]
Param.Nx = Param.col_length/Param.y_spacing; % number of trees in x direction
Param.Ny = 10;         % number of rows in y direction

% System parameters
l = 3; % Trailer length

% MPC Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_c = eye(2);
B_c = eye(2);
% Output is angular position (theta)
C_c = eye(2);
D_c = 0;

% Discretize
sysc = ss(A_c, B_c, C_c, D_c);
sysd = c2d(sysc, Param.dTi);
A = sysd.A;
B = sysd.B;
C = sysd.C;
nx = size(A,1);
nu = size(B,2);
ny = size(C,1);

% MPC parameters
N = 10;
P = eye(nu);
% The reference is in deg we need to have the cost in rads
Q = (360/2*pi)*eye(nu);
R = eye(nu);
F = eye(nx);
G = zeros(nx, N*nu);

Gaux = [];
for n = 1:N
    F = [F; A^n];
    Gaux = [A^(n-1)*B Gaux];
    G = [G; Gaux zeros(nx, (N-n)*nu)];
end

H = kron(eye(N+1), C);
F_bar = H*F;
G_bar = H*G;
Q_bar = blkdiag(kron(eye(N), Q), P);
R_bar = kron(eye(N), R);

R_til = G_bar'*Q_bar*G_bar+R_bar;
S_til = G_bar'*Q_bar;
Ky_full = R_til^(-1)*S_til;
Kx_full = Ky_full*F_bar;

MPC.Ky = Ky_full;
MPC.K = Kx_full;
% End of MPC Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%