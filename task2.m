load('mocapPoints3D.mat'); 

% assuming that the data is all in pixels 
% 3D world --> 3D camera
% Uses helper function projectPoints.m

load('Parameters_V1_1.mat');
[x, y] = projectPoints(pts3D, Parameters);
img = imread('im1corrected.jpg');  
figure;                     
imshow(img);                  
hold on;                      
plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
save('projectedPoints_cam1.mat', 'x', 'y');

load('Parameters_V2_1.mat');
[x, y] = projectPoints(pts3D, Parameters);
img = imread('im2corrected.jpg');  
figure;                     
imshow(img);                  
hold on;                      
plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
save('projectedPoints_cam2.mat', 'x', 'y');

