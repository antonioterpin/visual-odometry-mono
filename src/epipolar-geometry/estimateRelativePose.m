function [T, R, t] = estimateRelativePose(p1,p2,K1,K2)
% ESTIMATERELATIVEPOSE Estimates the pose of the second frame relative to
% the first one.
% 
% T = ESTIMATERELATIVEPOSE(p1,p2,K) Returns the 4x4 transformation matrix 
% from the first frame to the second.
% p1 is 3xN, the 2D homogenous coordinates in the first image. [u; v; 1]
% p2 is 3xN, the 2D homogenous coordinates in the second image. [u; v; 1]
%
% [..., R, t] = ESTIMATERELATIVEPOSE(...) Additionally returns the 3x3
% rotation matrix and the 3x1 translation vector.
%
% ESTIMATERELATIVEPOSE(..., K2) Allows for images taken with cameras with
% different intrinsics.

arguments
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2)}
    K1 (3,3)
    K2 (3,3) = K1
end

E = estimateEssentialMatrix(p1, p2, K1, K2);
[Rots,t] = decomposeEssentialMatrix(E);
[R,t] = disambiguatePose(Rots,t,p1,p2,K1,K2);

T = [R, t; 0, 0, 0, 1];
end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end
