%get set of 39 points from task2 for BOTH images
%for a F matrix, compute epipolar line in image2 for each point in image1 and vice versa
%compute the squared distance from each point to its corresponding epipolar line
%then compute the mean over all these squared distances
%using two F matrices: from task5 and 8-point algorithm (task6)
%verify that the error from task5 is lower than task6

load('mocapPoints3D.mat'); 
%points from image 1
load('Parameters_V1_1.mat');
[x1, y1] = projectPoints(pts3D, Parameters);

%points from image 2
load('Parameters_V2_1.mat');
[x2, y2] = projectPoints(pts3D, Parameters);

%load F matrices
load('task5_results.mat'); %F_corrected
load('task6_results.mat'); %F_eight
%normalize F matrices for comparison
corr_F = F_corrected / F_corrected(3,3)
eight_F = F_eight / F_eight(3,3)

%compute total squared distance for corr_F
sum_dist_corr1 = 0;
sum_dist_corr2 = 0;
sum_dist_eight1 = 0;
sum_dist_eight2 = 0;
for i = 1:length(x1)
    %get the points
    p1 = [x1(i); y1(i); 1];
    p2 = [x2(i); y2(i); 1];

    %corresponding epipolar line in opposite image for corrected F
    corr_l2 = corr_F * p1; 
    corr_l1 = corr_F' * p2; 

    %corresponding epipolar line in opposite image for eight-point F
    eight_l2 = eight_F * p1;
    eight_l1 = eight_F' * p2; 

    %distance from point to line for corrected F
    d2 = ((corr_l2(1)*p2(1))+(corr_l2(2)*p2(2))+corr_l2(3))^2 / (corr_l2(1)^2 + corr_l2(2)^2);
    d1 = ((corr_l1(1)*p1(1))+(corr_l1(2)*p1(2))+corr_l1(3))^2 / (corr_l1(1)^2 + corr_l1(2)^2);
    sum_dist_corr1 = sum_dist_corr1 + d1;
    sum_dist_corr2 = sum_dist_corr2 + d2;

    %distance from point to line for eight-point F
    d2_eight = ((eight_l2(1)*p2(1)) + (eight_l2(2)*p2(2)) + eight_l2(3))^2 / (eight_l2(1)^2 + eight_l2(2)^2);
    d1_eight = ((eight_l1(1)*p1(1)) + (eight_l1(2)*p1(2)) + eight_l1(3))^2 / (eight_l1(1)^2 + eight_l1(2)^2);
    sum_dist_eight1 = sum_dist_eight1 + d1_eight;
    sum_dist_eight2 = sum_dist_eight2 + d2_eight;
end

mean_dist_corr = (sum_dist_corr1 + sum_dist_corr2) / (2 * length(x1))
mean_dist_eight = (sum_dist_eight1 + sum_dist_eight2) / (2 * length(x1))


    

