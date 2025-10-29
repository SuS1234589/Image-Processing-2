% Lines in Image 2 don't cross the markers because cropping/geometry means their epipolar constraints lie outside the visible area. 
clear; clc;
im1 = imread('im1corrected.jpg');
im2 = imread('im2corrected.jpg');
load('Parameters_V1_1.mat');
K1 = Parameters.Kmat; R1 = Parameters.Rmat; C1 = Parameters.position(:); T1 = -R1*C1;
load('Parameters_V2_1.mat');
K2 = Parameters.Kmat; R2 = Parameters.Rmat; C2 = Parameters.position(:); T2 = -R2*C2;

crop_rect1 = [500 200 800 800];
crop_rect2 = [600 250 700 800];
im1_crop = imcrop(im1, crop_rect1);
im2_crop = imcrop(im2, crop_rect2);

xc1 = crop_rect1(1); yc1 = crop_rect1(2);
xc2 = crop_rect2(1); yc2 = crop_rect2(2);

K1_new = K1; K1_new(1,3) = K1(1,3) - xc1; K1_new(2,3) = K1(2,3) - yc1;
K2_new = K2; K2_new(1,3) = K2(1,3) - xc2; K2_new(2,3) = K2(2,3) - yc2;

P1_crop = K1_new * [R1 T1];
P2_crop = K2_new * [R2 T2];

R_rel = R2 * R1';
t_rel = C2 - C1;
tx = [0 -t_rel(3) t_rel(2); t_rel(3) 0 -t_rel(1); -t_rel(2) t_rel(1) 0];
E = tx * R_rel;
F_crop = inv(K2_new)' * E * inv(K1_new);
[Uf,Sf,Vf]=svd(F_crop); Sf(3,3)=0; F_crop=Uf*Sf*Vf';

disp('Cropped Fundamental matrix:');
disp(F_crop);

load('mocapPoints3D.mat'); % pts3D: 3 x N (e.g., N=39)
pts3D_hom = [pts3D; ones(1, size(pts3D,2))]; % 4 x N

proj1_full = K1 * [R1 T1] * pts3D_hom;
proj2_full = K2 * [R2 T2] * pts3D_hom;
u1_full = proj1_full(1,:) ./ proj1_full(3,:);
v1_full = proj1_full(2,:) ./ proj1_full(3,:);
u2_full = proj2_full(1,:) ./ proj2_full(3,:);
v2_full = proj2_full(2,:) ./ proj2_full(3,:);

u1_proj = u1_full - xc1;
v1_proj = v1_full - yc1;
u2_proj = u2_full - xc2;
v2_proj = v2_full - yc2;

% Show all points for context
figure; imshow(im1_crop); hold on; plot(u1_proj, v1_proj, 'ro', 'MarkerSize', 6, 'LineWidth', 2);
title('All Projected Points in Cropped Image 1');
figure; imshow(im2_crop); hold on; plot(u2_proj, v2_proj, 'go', 'MarkerSize', 6, 'LineWidth', 2);
title('All Projected Points in Cropped Image 2');

% Select ALL points for line visualization
sel_idx = 1:length(u1_proj); % 1:N, covers full spread
pts_crop1 = [u1_proj(sel_idx)', v1_proj(sel_idx)'];
pts_crop2 = [u2_proj(sel_idx)', v2_proj(sel_idx)'];

plot_epipolar_lines_debug(im1_crop, im2_crop, pts_crop1, pts_crop2, F_crop);

function plot_epipolar_lines_debug(im1, im2, pts1, pts2, F)
    figure;
    subplot(1,2,1); imshow(im1); hold on;
    sz1 = size(im1);
    cmap = lines(size(pts1,1)); % Distinct color per line/point
    for i = 1:size(pts1,1)
        l = F' * [pts2(i,:) 1]';
        %fprintf('Img1 Line %d: %.4f %.4f %.4f\n', i, l(1), l(2), l(3));
        x = 1:sz1(2);
        color = cmap(i,:);
        if abs(l(2)) > 1e-8
            y = (-l(1)*x - l(3))/l(2);
            plot(x, y, '-', 'Color', color, 'LineWidth', 2);
        else
            x_val = -l(3)/l(1);
            plot([x_val x_val], [1 sz1(1)], '-', 'Color', color, 'LineWidth', 2);
        end
        plot(pts1(i,1), pts1(i,2), 'o', 'Color', color, 'MarkerSize', 8, 'LineWidth', 2);
    end
    title('Epipolar lines in Cropped Image 1');

    subplot(1,2,2); imshow(im2); hold on;
    sz2 = size(im2);
    for i = 1:size(pts2,1)
        l = F * [pts1(i,:) 1]';
        %fprintf('Img2 Line %d: %.4f %.4f %.4f\n', i, l(1), l(2), l(3));
        x = 1:sz2(2);
        color = cmap(i,:);
        if abs(l(2)) > 1e-8
            y = (-l(1)*x - l(3))/l(2);
            plot(x, y, '-', 'Color', color, 'LineWidth', 2);
        else
            x_val = -l(3)/l(1);
            plot([x_val x_val], [1 sz2(1)], '-', 'Color', color, 'LineWidth', 2);
        end
        plot(pts2(i,1), pts2(i,2), 'o', 'Color', color, 'MarkerSize', 8, 'LineWidth', 2);
    end
    title('Epipolar lines in Cropped Image 2');
end