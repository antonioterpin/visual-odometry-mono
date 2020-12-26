function [validLandmarks, landmarks] = filterPoints(landmarks, R_CW, t_CW)
% FILTERPOINTS Filter landmarks that are actually behind the camera (and
% thus, not valid for sure if detected by the same camera).
%   
% [validLandmarks, landmarks] = FILTERPOINTS(landmarks, R_CW, t_CW) returns
% the logical indices of the valid landmarks and the filtered landmarks.
% landmarks is 3xN or 4xN, 3D (eventually homogenous) coordinates.
% R_CW is 3x3, the rotation matrix from the world to the camera frame.
% t_CW is 3x1, the translation vector from the world to the camera frame.

% Should be R_WC, t_WC

arguments
    landmarks {mustBeLandmarks(landmarks)}
    R_CW (3,3)
    t_CW (3,1)
end

global verbose;

validLandmarks = R_CW(3,1:3)*landmarks(1:3,:) > repmat(-t_CW(3), [1 size(landmarks, 2)]);
landmarks = landmarks(:,validLandmarks);

verboseDisp(verbose, '%d out of %d valid landmarks', ...
    [nnz(validLandmarks), length(validLandmarks)]);
end

function mustBeLandmarks(landmarks)
    if size(landmarks,1) ~= 3 && size(landmarks,1) ~= 4
        eid = 'Size:notEqual';
        msg = 'Landmarks must be either 3xN or 4xN.';
        throwAsCaller(MException(eid,msg))
    end
end



