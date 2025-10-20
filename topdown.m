% Accurate top-down floor view using both calibrated images

clear; clc;

% Load images and camera calibration
im1 = imread('im1corrected.jpg');
im2 = imread('im2corrected.jpg');

load('Parameters_V1_1.mat'); % loads struct Parameters
K1 = Parameters.Kmat; R1 = Parameters.Rmat;
C1 = Parameters.position(:); T1 = -R1*C1; P1 = K1*[R1,T1];

load('Parameters_V2_1.mat'); % loads struct Parameters
K2 = Parameters.Kmat; R2 = Parameters.Rmat;
C2 = Parameters.position(:); T2 = -R2*C2; P2 = K2*[R2,T2];

% Define metrically correct ground rectangle in mm
Xmin = -5000; Xmax = 5000;
Ymin = -5000; Ymax = 5000;
W = 2400; H = 2400; % output resolution

[Xg, Yg] = meshgrid(linspace(Xmin,Xmax,W), linspace(Ymin,Ymax,H));
Zg = zeros(size(Xg)); % Floor (Z=0)
pts3D = [Xg(:)'; Yg(:)'; Zg(:)'; ones(1,numel(Xg))];

% Project to both images
uv1 = P1 * pts3D; u1 = uv1(1,:)./uv1(3,:); v1 = uv1(2,:)./uv1(3,:);
uv2 = P2 * pts3D; u2 = uv2(1,:)./uv2(3,:); v2 = uv2(2,:)./uv2(3,:);

U1 = reshape(u1,H,W); V1 = reshape(v1,H,W);
U2 = reshape(u2,H,W); V2 = reshape(v2,H,W);

% Viewing direction for each pixel (from camera to ground point)
pts_floor = [Xg(:) Yg(:) Zg(:)];
dir_cam1 = pts_floor - C1'; % N x 3
dir_cam2 = pts_floor - C2';

% Floor normal is [0 0 1]
norm_floor = [0 0 1];

% Compute dot product (cosine of angle to floor normal)
dot1 = dir_cam1 * norm_floor'; % N x 1
dot2 = dir_cam2 * norm_floor';

% Reshape to grid
dot1 = reshape(dot1,H,W);
dot2 = reshape(dot2,H,W);

% For each pixel: choose best camera (max dot product, i.e., most direct view)
valid1 = (U1 > 1) & (U1 < size(im1,2)) & (V1 > 1) & (V1 < size(im1,1));
valid2 = (U2 > 1) & (U2 < size(im2,2)) & (V2 > 1) & (V2 < size(im2,1));

out = zeros(H, W, 3, 'uint8');
for c = 1:3
    % Sample images at projected coordinates
    warped1 = interp2(double(im1(:,:,c)), U1, V1, 'linear', 0);
    warped2 = interp2(double(im2(:,:,c)), U2, V2, 'linear', 0);

    % Blend logic:
    % 1) use camera that views more directly (max dot with floor normal)
    best1 = (valid1 & ((dot1 >= dot2) | ~valid2));
    best2 = (valid2 & ((dot2 > dot1) | ~valid1));
    both_valid = valid1 & valid2;
    
    fused = zeros(H,W);
    fused(best1) = warped1(best1);
    fused(best2) = warped2(best2);
    % If both valid and similar viewing angle, take average
    fused(both_valid & abs(dot1-dot2)<0.1*max(dot1(:))) = ...
        0.5*(warped1(both_valid & abs(dot1-dot2)<0.1*max(dot1(:))) + warped2(both_valid & abs(dot1-dot2)<0.1*max(dot1(:))));

    out(:,:,c) = uint8(fused);
end

figure; imshow(out);
title('Accurate Top-down Floor from Both Cameras');

% Overlay axes
hold on;
plot([1 W],[H/2 H/2],'r-','LineWidth',2);
plot([W/2 W/2],[1 H],'g-','LineWidth',2);
text(W-50, H/2, '+X', 'Color', 'r', 'FontSize', 12);
text(W/2, 30, '+Y', 'Color', 'g', 'FontSize', 12);

imwrite(out, 'top_down_fused.png');

% Optional: Save mm-to-pixel mapping for downstream use
save('top_down_fused_info.mat','Xmin','Xmax','Ymin','Ymax','W','H');
