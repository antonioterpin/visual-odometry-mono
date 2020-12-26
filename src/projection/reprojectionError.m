function error = reprojectionError(P_W, p_I, K, R_CW, t_CW)
%REPROJECTIONERROR Summary of this function goes here
%   TODO documentation
%   P_W 3xN   3D world points
%   p_I 2xN   image points
%   
% TODO arguments validation

points_C = R_CW * P_W + repmat(t_CW, [1, size(P_W, 2)]);
reprojection = projectPoints(points_C, K);
error = sum((p_I - reprojection).^2, 1);

end

