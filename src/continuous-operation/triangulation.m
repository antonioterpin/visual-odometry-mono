function [outputArg1,outputArg2] = triangulation(currPose, )
%TRIANGULATION Summary of this function goes here
%   Detailed explanation goes here
    % Triangulate new data
% if ~isempty(currPose)
    [triangulatedKeypoints, triangulatedLandmarks, sizeHistory, ...
        lastKeypoints, lastDescriptors] ...
            = triangulateNewData(K, keypoints, descriptors, lastKeypoints, ...
                keypointsMatched, lastDescriptors, sizeHistory, currPose);

%     if ~isempty(triangulatedKeypoints)
%         R_IC = reshape(currPose(1:9),3,3);
%         t_IC = reshape(currPose(10:12),3,1);
%         newState = [triangulatedKeypoints; triangulatedLandmarks(1:3,:)];
%         disp('NEW STATE ADDED WITH SUCCESS!!')
%         Add new Landmarks
%         currState = [currState, newState];
%         Filter out points not in front
%         poseW =-R_IC.'*t_IC; %world pose
%         inFront = R_IC(3,1:3)*(currState(3:5,:)-poseW) > 0;
%         currState = currState(:, inFront == 1);
%         disp([num2str(size(currState,2)) ' landmarks currently being tracked'])
%     end
% end
end

