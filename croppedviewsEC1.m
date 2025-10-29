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

load('mocapPoints3D.mat'); % pts3D: 3 x N (N = 39)
pts3D_hom = [pts3D; ones(1, size(pts3D,2))]; % 4 x N

% 3D projection to full image
proj1_full = K1 * [R1 T1] * pts3D_hom; % 3 x N
proj2_full = K2 * [R2 T2] * pts3D_hom; % 3 x N
u1_full = proj1_full(1,:) ./ proj1_full(3,:);
v1_full = proj1_full(2,:) ./ proj1_full(3,:);
u2_full = proj2_full(1,:) ./ proj2_full(3,:);
v2_full = proj2_full(2,:) ./ proj2_full(3,:);

% 3D projection to crop by subtracting crop origin
u1_proj = u1_full - xc1;
v1_proj = v1_full - yc1;
u2_proj = u2_full - xc2;
v2_proj = v2_full - yc2;

figure; imshow(im1_crop); hold on; plot(u1_proj, v1_proj, 'r+'); title('Cropped 1, Projected Points');
figure; imshow(im2_crop); hold on; plot(u2_proj, v2_proj, 'g+'); title('Cropped 2, Projected Points');

sel_idx = 1:10; % Use some points for epipolar line demo
pts_crop1 = [u1_proj(sel_idx)', v1_proj(sel_idx)'];
pts_crop2 = [u2_proj(sel_idx)', v2_proj(sel_idx)'];

% Clip points to be inside crop for plotting:
w1 = size(im1_crop,2); h1 = size(im1_crop,1);
w2 = size(im2_crop,2); h2 = size(im2_crop,1);
pts_crop1(:,1) = max(1, min(w1, pts_crop1(:,1)));
pts_crop1(:,2) = max(1, min(h1, pts_crop1(:,2)));
pts_crop2(:,1) = max(1, min(w2, pts_crop2(:,1)));
pts_crop2(:,2) = max(1, min(h2, pts_crop2(:,2)));

plot_epipolar_lines_cropped(im1_crop, im2_crop, pts_crop1, pts_crop2, F_crop);

function plot_epipolar_lines_cropped(im1, im2, pts1, pts2, F)
    subplot(1,2,1); imshow(im1); hold on;
    sz = size(im1);
    for i = 1:size(pts1,1)
        l = F' * [pts2(i,:) 1]';
        if abs(l(2)) > 1e-8
            x = 1:sz(2);
            y = (-l(1)*x - l(3))/l(2);
            valid = (y >= 1) & (y <= sz(1));
            plot(x(valid), y(valid), 'r-');
        else
            x = repmat(-l(3)/l(1), 1, 2); y = [1 sz(1)];
            plot(x, y, 'r-');
        end
        plot(pts1(i,1), pts1(i,2),'go');
    end
    title('Epipolar lines in Cropped Image 1');

    subplot(1,2,2); imshow(im2); hold on;
    sz = size(im2);
    for i = 1:size(pts2,1)
        l = F * [pts1(i,:) 1]';
        if abs(l(2)) > 1e-8
            x = 1:sz(2);
            y = (-l(1)*x - l(3))/l(2);
            valid = (y >= 1) & (y <= sz(1));
            plot(x(valid), y(valid), 'b-');
        else
            x = repmat(-l(3)/l(1), 1, 2); y = [1 sz(1)];
            plot(x, y, 'b-');
        end
        plot(pts2(i,1), pts2(i,2),'go');
    end
    title('Epipolar lines in Cropped Image 2');
end
