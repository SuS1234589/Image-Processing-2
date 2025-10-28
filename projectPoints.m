%NEED TO MAKE THIS A SERPATE FUNCTION SO THAT IT CAN BE CALLED FROM OTHER SCRIPTS
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