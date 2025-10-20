% Task 3.4: Automated Point Selection and Triangulation
% This script loads images and camera parameters, then uses interactive
% or pre-defined point selection for triangulation measurements

clear; clc; close all;

% Load camera parameters (adjust filenames as needed)
load('Parameters_V1_1.mat'); % Should contain Camera1 struct

K1 = Parameters.Kmat;
R1 = Parameters.Rmat;

% Camera location/translation
C1 = Parameters.position(:); % Camera center in world coords
T1 = -R1 * C1; % Translation vector

% Build projection matrix for Camera 1
P1 = K1 * [R1, T1];

load('Parameters_V2_1.mat'); % Should contain Camera2 struct

K2 = Parameters.Kmat;
R2 = Parameters.Rmat;

% Camera location/translation
C2 = Parameters.position(:); % Camera center in world coords
T2 = -R2 * C2; % Translation vector

% Build projection matrix for Camera 1
P2 = K2 * [R2, T2];


% Load corrected images
im1 = imread('im1corrected.jpg');
im2 = imread('im2corrected.jpg');

% Display images side by side for reference
figure('Name', 'Image Pair for Point Selection');
subplot(1,2,1); imshow(im1); title('Image 1');
subplot(1,2,2); imshow(im2); title('Image 2');

% Interactive Point Selection


fprintf('=== POINT SELECTION INSTRUCTIONS ===\n');
fprintf('Select points in BOTH images in the SAME order:\n');
fprintf('1-3:   Three points on the FLOOR\n');
fprintf('4-6:   Three points on the WALL with stripes\n');
fprintf('7-8:   DOORWAY - top edge and bottom edge\n');
fprintf('9-10:  PERSON - top of head and feet position\n');
fprintf('11:    CAMERA on tripod location\n\n');

% Use cpselect for point selection
[points_img1, points_img2] = cpselect(im1, im2, 'Wait', true);

% Verify we have at least 11 points
if size(points_img1,1) < 11
    error('Need at least 11 point pairs! You selected %d', size(points_img1,1));
end

% Triangulation Function
function X = triangulate_point(P1, P2, x1, x2)
    % Direct Linear Transform (DLT) triangulation
    % P1, P2: 3x4 projection matrices
    % x1, x2: [u v] image coordinates
    % X: 3D point in world coordinates
    
    A = [x1(1)*P1(3,:) - P1(1,:);
         x1(2)*P1(3,:) - P1(2,:);
         x2(1)*P2(3,:) - P2(1,:);
         x2(2)*P2(3,:) - P2(2,:)];
    
    [~,~,V] = svd(A);
    X_homog = V(:,end);
    X = X_homog(1:3) / X_homog(4);
end

% Triangulate All Points
N_points = size(points_img1, 1);
points_3d = zeros(N_points, 3);

fprintf('\nTriangulating %d points...\n', N_points);
for i = 1:N_points
    points_3d(i,:) = triangulate_point(P1, P2, points_img1(i,:), points_img2(i,:))';
end

fprintf('Triangulation complete!\n\n');

% Plane Fitting Function
function plane_params = fit_plane(pts)
    % Fit plane to 3D points: ax + by + cz + d = 0
    % Returns [a, b, c, d]
    
    centroid = mean(pts, 1);
    pts_centered = pts - centroid;
    
    [~, ~, V] = svd(pts_centered, 0);
    normal = V(:, 3)';  % Normal vector [a, b, c]
    
    d = -dot(normal, centroid);
    plane_params = [normal, d];
end

% Measurements

% 1. Floor Plane (points 1-3)
floor_pts = points_3d(1:3, :);
plane_floor = fit_plane(floor_pts);

fprintf('=== FLOOR PLANE ===\n');
fprintf('Floor plane equation: %.4fx + %.4fy + %.4fz + %.4f = 0\n', ...
    plane_floor(1), plane_floor(2), plane_floor(3), plane_floor(4));
fprintf('Floor normal: [%.4f, %.4f, %.4f]\n', plane_floor(1:3));
fprintf('If Z=0 is floor, expect normal close to [0, 0, ±1]\n\n');

% 2. Wall Plane (points 4-6)
wall_pts = points_3d(4:6, :);
plane_wall = fit_plane(wall_pts);

fprintf('=== WALL PLANE ===\n');
fprintf('Wall plane equation: %.4fx + %.4fy + %.4fz + %.4f = 0\n', ...
    plane_wall(1), plane_wall(2), plane_wall(3), plane_wall(4));
fprintf('Wall normal: [%.4f, %.4f, %.4f]\n\n', plane_wall(1:3));

% 3. Doorway Height (points 7-8)
door_top = points_3d(7, :);
door_bottom = points_3d(8, :);
door_height = norm(door_top - door_bottom);

fprintf('=== DOORWAY MEASUREMENTS ===\n');
fprintf('Door top point: [%.2f, %.2f, %.2f]\n', door_top);
fprintf('Door bottom point: [%.2f, %.2f, %.2f]\n', door_bottom);
fprintf('Doorway height: %.2f units\n\n', door_height);

% 4. Person Height (points 9-10)
head_top = points_3d(9, :);
feet = points_3d(10, :);
person_height = norm(head_top - feet);
person_height_z = abs(head_top(3) - feet(3)); % Vertical component only

fprintf('=== PERSON MEASUREMENTS ===\n');
fprintf('Head top point: [%.2f, %.2f, %.2f]\n', head_top);
fprintf('Feet point: [%.2f, %.2f, %.2f]\n', feet);
fprintf('Person height (3D distance): %.2f units\n', person_height);
fprintf('Person height (Z-component): %.2f units\n\n', person_height_z);

% 5. Camera Position (point 11)
camera_pos = points_3d(11, :);

fprintf('=== CAMERA POSITION ===\n');
fprintf('Camera on tripod at: [%.2f, %.2f, %.2f]\n\n', camera_pos);

%% Visualization
figure('Name', '3D Point Visualization');
hold on; grid on; axis equal;

% Plot floor points
plot3(floor_pts(:,1), floor_pts(:,2), floor_pts(:,3), ...
    'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Floor');

% Plot wall points  
plot3(wall_pts(:,1), wall_pts(:,2), wall_pts(:,3), ...
    'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Wall');

% Plot doorway
plot3([door_top(1) door_bottom(1)], [door_top(2) door_bottom(2)], ...
    [door_top(3) door_bottom(3)], 'g-', 'LineWidth', 3, 'DisplayName', 'Doorway');

% Plot person
plot3([head_top(1) feet(1)], [head_top(2) feet(2)], ...
    [head_top(3) feet(3)], 'm-', 'LineWidth', 3, 'DisplayName', 'Person');

% Plot camera
plot3(camera_pos(1), camera_pos(2), camera_pos(3), ...
    'k^', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'DisplayName', 'Camera');

xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Triangulated Points');
legend('Location', 'best');
view(3);

% Save Results
results = struct();
results.points_img1 = points_img1;
results.points_img2 = points_img2;
results.points_3d = points_3d;
results.plane_floor = plane_floor;
results.plane_wall = plane_wall;
results.door_height = door_height;
results.person_height = person_height;
results.camera_position = camera_pos;

save('task34_results.mat', 'results');
fprintf('Results saved to task34_results.mat\n');
