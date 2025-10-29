clear; clc;

% Load Camera 1 params
load('Parameters_V1_1.mat'); % struct Parameters
K1 = Parameters.Kmat;
R1 = Parameters.Rmat;
C1 = Parameters.position(:);           % Camera center in world coordinates
T1 = -R1 * C1;                        % Translation vector (world to camera)

% Load Camera 2 params
load('Parameters_V2_1.mat');
K2 = Parameters.Kmat;
R2 = Parameters.Rmat;
C2 = Parameters.position(:);
T2 = -R2 * C2;

% Projection matrices
P1 = K1 * [R1, T1];
P2 = K2 * [R2, T2];

% Relative rotation and translation from cam1 to cam2
R_rel = R2 * R1';
t_rel = R2 * (C1 - C2);  % Corrected translation in second camera's frame

% Skew symmetric matrix for t_rel
tx = [ 0      -t_rel(3)  t_rel(2);
       t_rel(3)  0      -t_rel(1);
      -t_rel(2)  t_rel(1) 0 ];

% Essential matrix
E = tx * R_rel;

% Fundamental matrix
F = inv(K2)' * E * inv(K1);

% Enforce rank-2 constraint
[U, S, V] = svd(F);
S(3,3) = 0;
F_corrected = U * S * V';

disp('Fundamental matrix F (rank 2 enforced):');
disp(F_corrected);

% Save for reuse
save('task5_results.mat', 'F', 'F_corrected', 'K1', 'K2', 'R1', 'R2', 't_rel', 'P1', 'P2');
