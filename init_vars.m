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