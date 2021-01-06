% The requested format is: 
%% Kitti
fid = fopen(sprintf('00.txt'));
fidTimes = fopen(sprintf('times.txt'));
fileID = fopen('stamped_groundtruth.txt', 'w');
fileIDEstimate = fopen('stamped_traj_estimate.txt', 'w');
line = fgetl(fid);
% Write estimate
for i = 1:1079
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
for i = 1:1095
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
    if ~(mod(i,59) == 0)
        fprintf(fileID, '%f %f %f %f %f %f %f %f\n', tmpTime, t(1), t(2), t(3), ...
            q(2), q(3), q(4), q(1));
    end
end
disp('Written out Ground-Truth.');
%% Malaga
%GT
%Create collection of all GPS positions
historyPosesMalaga = [];
fileIDEstimate = fopen('stamped_traj_estimate.txt', 'w');
fid = fopen(sprintf('malaga-urban-dataset-extract-07_all-sensors_GPS.txt'));
line = fgetl(fid);  %skip first line
for i = 2 : 107
    line = fgetl(fid);
    tmpPose = sscanf(line, '%f');
    pose = [-tmpPose(9);
        -tmpPose(11);
        -tmpPose(10)];
    historyPosesMalaga = [historyPosesMalaga, pose];
end
%Fit positions and generate new datapoints
t = 1 : numel(historyPosesMalaga(1, :));
xy = [historyPosesMalaga(1, :);
    historyPosesMalaga(3, :)];
pp = spline(t, xy);
tInterp = linspace(1, numel(historyPosesMalaga(1, :)), 2121);
xyInterp = ppval(pp, tInterp);
%again for Y
xy = [historyPosesMalaga(1, :);
    historyPosesMalaga(2, :)];
pp = spline(t, xy);
tInterp = linspace(1, numel(historyPosesMalaga(1, :)), 2121);
xyInterp1 = ppval(pp, tInterp);
discretizedPosesMalaga = [xyInterp(1, :);
                            xyInterp1(2, :);
                            xyInterp(2, :)];
fclose(fid);
fileID = fopen('stamped_groundtruth.txt', 'w');
time = 1:2121;
q = [1,0,0,0];
t = discretizedPosesMalaga';
for i = 1 : 2000
    if ~(mod(i,3) == 0) && ~(mod(i,7) == 0)
        if ~any(isnan(t(i, :))) && ~any(isinf(t(i, :)))
    fprintf(fileID, '%f %f %f %f %f %f %f %f\n', time(i), t(i,1), t(i,2), t(i,3), ...
            q(2), q(3), q(4), q(1));
        end
    end
end
%Estimate
for i = 1:916
    rotEstimate = reshape(rots(i, :), [3,3]);
    rotEstimate = rotEstimate.';
    tE = -rotEstimate * trasl(i, 1:3).';
    qE = rotMatrix2Quat(rotEstimate);
    if ~any(isnan(tE)) && ~any(isinf(tE))
        if ~any(isnan(qE)) && ~any(isinf(qE))
            fprintf(fileIDEstimate, '%f %f %f %f %f %f %f %f\n', time(i), tE(1),...
                tE(2), tE(3), qE(2), qE(3), qE(4), qE(1));
        end
    end
end
disp('Written out Ground-Truth.');