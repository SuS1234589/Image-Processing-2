% Here, we verify that R and the position are multiplying correctly
% We do this by calculating rotational matrix (R) * offset matrix (using given position)
function [tee_matrix] = notices_ur_p(image)
    col = zeros(3,1);
    row = zeros(1,4);
    row(1,4) = 1;
    image_r = [image.Parameters.Rmat, col];
    rotational_matrix = [image_r; row];

    offset_matrix = eye(4);
    offset_matrix(1:3,4) = image.Parameters.position;

    tee_matrix = rotational_matrix * offset_matrix;
end



%check V1 first
image_one = load('Parameters_V1_1.mat');
ur_tee = notices_ur_p(image_one);
fprintf('Image V1\n');
fprintf('The calculated T matrix is:\n');
fprintf('%g\t%g\t%g\t%g\n', ur_tee');
fprintf('The given Pmat is:\n');
fprintf('%g\t%g\t%g\t%g\n', image_one.Parameters.Pmat');

%then check V@
image_two = load('Parameters_V2_1.mat');
ur_tee = notices_ur_p(image_two);
fprintf('\nImage V2\n');
fprintf('The calculated T matrix is:\n');
fprintf('%g\t%g\t%g\t%g\n', ur_tee');
fprintf('The given Pmat is:\n');
fprintf('%g\t%g\t%g\t%g\n', image_two.Parameters.Pmat');