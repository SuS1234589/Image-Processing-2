load('Parameters_V1_1.mat');
K = Parameters.Kmat;
R = Parameters.Rmat;    
C = Parameters.position;
T = -R * C';
P1 = K * [R, T];

load('projectedPoints_cam1.mat'); 
x1 = x; y1 = y;
matchedPoints1 = [x1, y1];

load('Parameters_V2_1.mat');
K = Parameters.Kmat;
R = Parameters.Rmat;    
C = Parameters.position;
T = -R * C';
P2 = K * [R, T];

load('projectedPoints_cam2.mat'); 
x2 = x; y2 = y;
matchedPoints2 = [x2, y2];

worldPoints = triangulate(matchedPoints1, matchedPoints2, P1', P2');

% Compare to original 3D points
load('mocapPoints3D.mat');
if size(pts3D,1)==3
    pts3D = pts3D';
end
mse = mean(sum((worldPoints - pts3D).^2, 2));
fprintf('Mean squared error: %.6f\n', mse);
% It is same we good 