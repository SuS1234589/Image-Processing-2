load('mocapPoints3D.mat'); 

% assuming that the data is all in pixels 
% 3D world --> 3D camera
function [x, y] = projectPoints(pts3D, Parameters)
R = Parameters.Rmat;
C = Parameters.position;
T = -R * C';
N = size(pts3D, 2);
p_cam_all = zeros(N, 3);

for i = 1:N
    p_world = pts3D(:, i);
    p_cam_all(i, :) = (R * p_world + T)';
end

% 3D Camera --> 2D Film
f = Parameters.foclen;
X = p_cam_all(:, 1);
Y = p_cam_all(:, 2);
Z = p_cam_all(:, 3);
x_film = f * (X ./ Z);
y_film = f * (Y ./ Z);

% 2D Film --> 2D Pixel 
ox = Parameters.prinpoint(1);
oy = Parameters.prinpoint(2);
x = x_film + ox; % u
y = y_film + oy; % v

end 
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
