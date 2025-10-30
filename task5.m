clear; clc;

im1 = imread('im1corrected.jpg');
im2 = imread('im2corrected.jpg');

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

num_pts = 8;
[h1,w1,~] = size(im1);
[h2,w2,~] = size(im2);
points1 = [randi([1 w1], num_pts,1), randi([1 h1], num_pts,1)];
points2 = [randi([1 w2], num_pts,1), randi([1 h2], num_pts,1)];

figure; imshow(im1); hold on;
plot(points1(:,1), points1(:,2), 'r+');
title('Points in Image 1 with Epipolar Lines in Image 2');

figure; imshow(im2); hold on;
plot(points2(:,1), points2(:,2), 'g+');
title('Points in Image 2 with Epipolar Lines in Image 1');

% Function to plot epipolar lines
plot_epipolar_lines(im1, im2, points1, points2, F_corrected);

% Plot epipolar lines visualization function
function plot_epipolar_lines(im1, im2, pts1, pts2, F)
    figure;
    subplot(1,2,1); imshow(im1); hold on;
    sz1 = size(im1);
    for i = 1:size(pts1,1)
        l = F' * [pts2(i,:) 1]';
        x = 1:sz1(2);
        if abs(l(2)) > 1e-10
            y = (-l(1)*x - l(3))/l(2);
            valid = (y>=1 & y<=sz1(1));
            plot(x(valid), y(valid), 'r-', 'LineWidth', 1.5);
        else
            xval = -l(3)/l(1);
            plot([xval xval], [1 sz1(1)], 'r-', 'LineWidth', 1.5);
        end
        plot(pts1(i,1), pts1(i,2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    end
    title('Epipolar lines in Image 1');

    subplot(1,2,2); imshow(im2); hold on;
    sz2 = size(im2);
    for i = 1:size(pts2,1)
        l = F * [pts1(i,:) 1]';
        x = 1:sz2(2);
        if abs(l(2)) > 1e-10
            y = (-l(1)*x - l(3))/l(2);
            valid = (y>=1 & y<=sz2(1));
            plot(x(valid), y(valid), 'b-', 'LineWidth', 1.5);
        else
            xval = -l(3)/l(1);
            plot([xval xval], [1 sz2(1)], 'b-', 'LineWidth', 1.5);
        end
        plot(pts2(i,1), pts2(i,2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    end
    title('Epipolar lines in Image 2');
end