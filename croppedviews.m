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
disp(F_crop);

load('mocapPoints3D.mat'); % Load Nx3 array pts3D



% Make homogeneous coordinates (4 x N)
pts3D_hom = [pts3D, ones(size(pts3D,1), 2)];
disp(size(P1_crop));    % Should be 3 x 4
disp(size(pts3D));
disp(size(pts3D_hom));  % Should be 4 x N

proj1 = P1_crop * pts3D_hom; % 3 x N


% Projection
proj1 = P1_crop * pts3D_hom; % 3 x N
u1_proj = proj1(1,:) ./ proj1(3,:);
v1_proj = proj1(2,:) ./ proj1(3,:);

proj2 = P2_crop * pts3D_hom; % 3 x N
u2_proj = proj2(1,:) ./ proj2(3,:);
v2_proj = proj2(2,:) ./ proj2(3,:);



figure; imshow(im1_crop); hold on; plot(u1_proj, v1_proj, 'r+');
figure; imshow(im2_crop); hold on; plot(u2_proj, v2_proj, 'g+');

sel_idx = 1:10;
pts_crop1 = [u1_proj(sel_idx)', v1_proj(sel_idx)'];
pts_crop2 = [u2_proj(sel_idx)', v2_proj(sel_idx)'];
plot_epipolar_lines_cropped(im1_crop, im2_crop, pts_crop1, pts_crop2, F_crop);

function plot_epipolar_lines_cropped(im1, im2, pts1, pts2, F)
    figure;
    subplot(1,2,1); imshow(im1); hold on;
    for i = 1:size(pts1,1)
        l = F' * [pts2(i,:) 1]';
        x = 1:size(im1,2); y = (-l(1)*x - l(3))/l(2);
        plot(x, y, 'r'); plot(pts1(i,1), pts1(i,2),'go');
    end
    title('Epipolar lines in Cropped Image 1');
    subplot(1,2,2); imshow(im2); hold on;
    for i = 1:size(pts2,1)
        l = F * [pts1(i,:) 1]';
        x = 1:size(im2,2); y = (-l(1)*x - l(3))/l(2);
        plot(x, y, 'b'); plot(pts2(i,1), pts2(i,2),'go');
    end
    title('Epipolar lines in Cropped Image 2');
end
