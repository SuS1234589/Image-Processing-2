load('task34_results.mat'); % loads 'results' struct

pts1 = results.points_img1; % Correspondences in Image 1
pts2 = results.points_img2; % Correspondences in Image 2


% Load images
im1 = imread('im1corrected.jpg');
im2 = imread('im2corrected.jpg');

N = size(pts1, 1);

% Normalize image points for numerical stability
[norm_pts1, T1] = normalizePoints2d(pts1);
[norm_pts2, T2] = normalizePoints2d(pts2);

% Build matrix A for the algorithm
A = [ norm_pts2(:,1).*norm_pts1(:,1), ...
      norm_pts2(:,1).*norm_pts1(:,2), ...
      norm_pts2(:,1), ...
      norm_pts2(:,2).*norm_pts1(:,1), ...
      norm_pts2(:,2).*norm_pts1(:,2), ...
      norm_pts2(:,2), ...
      norm_pts1(:,1), norm_pts1(:,2), ...
      ones(N,1) ];

% SVD to solve Af=0
[~, ~, V] = svd(A,0);
Fhat = reshape(V(:,end), [3 3])';

% Enforce rank 2 constraint on Fhat
[Uf, Sf, Vf] = svd(Fhat);
Sf(3,3) = 0;
Fhat = Uf * Sf * Vf';

% Denormalize
F_eight = T2' * Fhat * T1;

disp('Eight-point Fundamental matrix:');
disp(F_eight);

% Plot epipolar lines using helper function
plot_epipolar_lines(im1, im2, pts1, pts2, F_eight);

% ===== Helper: Point Normalization =====
function [norm_pts, T] = normalizePoints2d(pts)
    centroid = mean(pts);
    dists = sqrt(sum((pts - centroid).^2, 2));
    mean_dist = mean(dists);
    scale = sqrt(2) / mean_dist;
    T = [scale 0 -scale*centroid(1);
         0 scale -scale*centroid(2);
         0 0 1];
    norm_pts_hom = (T * [pts ones(size(pts,1),1)]')';
    norm_pts = norm_pts_hom(:,1:2) ./ norm_pts_hom(:,3);
end

% ===== Helper: Epipolar Line Plotting =====
function plot_epipolar_lines(im1, im2, pts1, pts2, F)
    figure;
    subplot(1,2,1); imshow(im1); hold on;
    for i = 1:size(pts1,1)
        l = F' * [pts2(i,:) 1]';
        x = 1:size(im1,2); y = (-l(1)*x - l(3))/l(2);
        plot(x,y,'r'); plot(pts1(i,1), pts1(i,2),'go');
    end
    title('Epipolar lines in Image 1');
    subplot(1,2,2); imshow(im2); hold on;
    for i = 1:size(pts2,1)
        l = F * [pts1(i,:) 1]';
        x = 1:size(im2,2); y = (-l(1)*x - l(3))/l(2);
        plot(x,y,'b'); plot(pts2(i,1), pts2(i,2),'go');
    end
    title('Epipolar lines in Image 2');
end



