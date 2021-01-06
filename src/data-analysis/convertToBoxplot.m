% The requested format is: 
%% Kitti
fid = fopen(sprintf('00.txt'));
fidTimes = fopen(sprintf('times.txt'));
fileID = fopen('stamped_groundtruth.txt', 'w');
fileIDEstimate = fopen('stamped_traj_estimate.txt', 'w');
line = fgetl(fid);
% Write estimate
for i = 1:907
    lineTimes = fgetl(fidTimes);
    tmpTime = sscanf(lineTimes, '%f');
    rotEstimate = reshape(rots(i, :), [3,3]);
    rotEstimate = rotEstimate.';
    tE = -rotEstimate * trasl(i, 1:3).';
    qE = rotMatrix2Quat(rotEstimate);
    fprintf(fileIDEstimate, '%f %f %f %f %f %f %f %f\n', tmpTime, tE(1),...
        tE(2), tE(3), qE(2), qE(3), qE(4), qE(1)); 
end
% Write GT
fidTimes = fopen(sprintf('times.txt'));
for i = 1:930
    lineTimes = fgetl(fidTimes);
    tmpTime = sscanf(lineTimes, '%f');
    line = fgetl(fid);
    tmpPose = sscanf(line, '%f');
    t = [tmpPose(4);
        tmpPose(8);
        tmpPose(12)];
    rotation = [tmpPose(1:3)';
        tmpPose(5:7)';
        tmpPose(9:11)'];
    q = rotMatrix2Quat(rotation);
    if ~(mod(i,49) == 0)
        fprintf(fileID, '%f %f %f %f %f %f %f %f\n', tmpTime, t(1), t(2), t(3), ...
            q(2), q(3), q(4), q(1));
    end
end
disp('Written out Ground-Truth.');

