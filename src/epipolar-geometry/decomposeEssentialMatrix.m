function [Rots,t] = decomposeEssentialMatrix(E)
% DECOMPOSEESSENTIALMATRIX Extract the possible rotation matrices and the
% translation (up to a sign) from the given essential matrix.
%
% [Rots, t] = DECOMPOSEESSENTIALMATRIX(E) Rots(:,:,1) and Rots(:,:,2) are
% the two possible rotation matrices, while t and -t are the possible
% translation vectors. The essential matrix E can be obtained, for instance,
% with estimateEssentialMatrix. To disambiguate consider using the function
% disambiguateRelativePose.
%
% Example:
% K1,K2 are the intrinsic matrices of the calibrated cameras. p1 and p2 are
% the corresponding matches [u; v; 1].
% E = estimateEssentialMatrix(p1, p2, K1, K2);
% [Rots,t] = decomposeEssentialMatrix(E);
% [R,t] = disambiguatePose(Rots,t,p0,p1,K1,K2);
% R is the most likely rotation matrix and t the most likely translation
% vector. 
% T_21 = [R t; 0 0 0 1] is the transformation from frame 1 to frame 2.
%
% See also DISAMBIGUAPOSE, ESTIMATEESSENTIALMATRIX

arguments
    E (3,3)
end

[U,~,V] = svd(E);

% Translation
t = U(:,3);

% Rotations
W = [0 -1 0; 1 0 0; 0 0 1];
Rots(:,:,1) = U*W*V.';
Rots(:,:,2) = U*W.'*V.';
for i = 1:2 % TODO parfor
    Rots(:,:,i) = Rots(:,:,i) * det(Rots(:,:,i)); % to enforce det = 1
end

if norm(t) ~= 0
    t = normalize(t, 'norm');
end

end
