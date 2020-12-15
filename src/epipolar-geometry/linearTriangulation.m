function P = linearTriangulation(p1, p2, M1, M2)
% LINEARTRIANGULATION Triangulate the image points from two image frames.
%
% P = LINEARTRIANGULATION(p1,p2,M1,M2) triangulates the image
% correspondences p1 and p2, given the projection matrices M1 and M2.
% p1 and p2 are 3xN, the homogenous coordinates of the correspondences in
% the two image frames [u; v; 1].
% M1 and M2 are 3x4, the projection matrices.

arguments
    p1 (3,:)
    p2 {mustBeEqualSize(p1,p2)}
    M1 (3,4)
    M2 (3,4)
end

N = size(p1, 2);
P = zeros(4, N);

A1 = cross2Matrix(p1);
A2 = cross2Matrix(p2);
% TODO maybe it can be optimized
% q = num2cell(A1(:,:,1:2),[1,2]); bigA1 = blkdiag(q{:});
% bigM1 = kron(eye(N), M1);
% A = [bigA1 * M1 bigA2 * M2];
% SVD???
for j = 1 : N
    A = [A1(:,:,j)*M1; A2(:,:,j)*M2];
    [~,~,V] = svd(A,0);
    P(:,j) = V(:,end);
end

P = P ./ repmat(P(4,:),4,1);

end

function mustBeEqualSize(a,b)
    if ~isequal(size(a),size(b))
        eid = 'Size:notEqual';
        msg = 'Size of first input must equal size of second input.';
        throwAsCaller(MException(eid,msg))
    end
end


