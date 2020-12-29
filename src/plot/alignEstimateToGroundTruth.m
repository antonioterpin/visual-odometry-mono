function p_GC = alignEstimateToGroundTruth(pp_GC, p_VC)
% Returns the points of the estimated trajectory p_V_C transformed into the
% ground truth frame G. The similarity transform is chosen to be
% such that it results in the lowest error between the aligned trajectory
% points p_GC and the points of the ground truth trajectory pp_GC. All
% matrices are 3xN.
%
% Algebraic solution
% [R_GV, t_GV; 0, 1] = T_GV = minarg (pp_GC - T_GV * p_VC)
% We want to approximately solve pp_GC - T_GV * p_VC = 0
% This can be done in an "exact" way
% R_GV * p_VC + t_GV - pp_GC = 0 
% <=> p_VC.' * R_GV.' + t_GV.' = pp_GC.'
% <=> [p_VC.' 1] * [R_GV.'; t_GV.'] = pp_GC.'
% <=> Q*H = B, Q is Nx4, H is 4x3 and B is Nx3

Q = [p_VC.', ones(size(p_VC, 2), 1)];
B = pp_GC.';

H = pinv(Q) * B;
R_GV = H(1:3,1:3).';
t_GV = H(4,1:3).';

p_GC = R_GV * p_VC + t_GV;

end