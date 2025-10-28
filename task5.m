% Task 3.5: Compute Fundamental Matrix from Calibration (Corrected Load)

clear; clc;

% Load Camera 1 parameters
load('Parameters_V1_1.mat'); % Loads struct Parameters with camera info
K1 = Parameters.Kmat;
R1 = Parameters.Rmat;
C1 = Parameters.position(:);        % Camera center in world coords
T1 = -R1 * C1;                     % Translation vector
P1 = K1 * [R1, T1];                % Projection matrix for camera 1

% Load Camera 2 parameters
load('Parameters_V2_1.mat'); % Loads struct Parameters
K2 = Parameters.Kmat;
R2 = Parameters.Rmat;
C2 = Parameters.position(:);        % Camera center in world coords
T2 = -R2 * C2;                     % Translation vector
P2 = K2 * [R2, T2];                % Projection matrix for camera 2

% Compute relative orientation and translation between cameras
R_rel = R2 * R1';
t_rel = C2 - C1;

% Skew symmetric matrix from translation vector
tx = [0 -t_rel(3) t_rel(2);
      t_rel(3) 0 -t_rel(1);
     -t_rel(2) t_rel(1) 0];

% Compute essential matrix
E = tx * R_rel;

% Compute fundamental matrix
F = inv(K2)' * E * inv(K1);

% Enforce rank 2 constraint (correct numerical rank)
[U, S, V] = svd(F);
S(3,3) = 0;
F_corrected = U * S * V';

% Display fundamental matrix
disp('Fundamental matrix F (rank 2 enforced):');
disp(F_corrected);

% Save matrices for further use
save('task5_results.mat', 'F', 'F_corrected', 'K1', 'K2', 'R1', 'R2', 't_rel');
