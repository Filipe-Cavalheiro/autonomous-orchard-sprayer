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
Param.Ny = 4;         % number of rows in y direction

% System parameters
linkage_dim = 3; % Trailer length

A = [1  Param.dTi  0  0;
     0  1   0  0;
     0  0   1  Param.dTi;
     0  0   0  1];

B = [0.5*Param.dTi^2  0;
     Param.dTi        0;
     0     0.5*Param.dTi^2;
     0     Param.dTi];
C = eye(4); 

nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);

% MPC Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MPC parameters
N = 4;
P = eye(ny);
Q = eye(ny);
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

Ky = R_til^(-1)*S_til;
K = Ky*F_bar;

% End of MPC Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%