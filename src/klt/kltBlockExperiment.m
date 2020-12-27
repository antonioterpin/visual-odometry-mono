verboseDisp(obj.verbose, ...
    '\n\nProcessing frame %d\n=====================\n', ii);
if obj.justInitialized
    [landmarksState, ~, ~] ...
    = obj.state.getObservations(ii - obj.nSkip);
else
    landmarksState = trackedLandmarks;
end


[~, landmarksIdx, keypointsState] ...
    = obj.state.getObservations(ii - obj.nSkip);

%Klt
% TODO Ho provato ad creare nuovi keypoints da trackare ma la cosa non
% sembra voglia funzionare. Risolvere errori e capire se sia la strada
% giusta o meno. Daje!

%Generate new keypoints to track every time
image = inputHandler.getImage(ii);
[keypointsNew, ~] = obj.coBlock.Detector.extractFeatures(image);   %2000 keypoints is too much to track!!!
keypointsState = keypointsState / 4;
keypointsNew = keypointsNew(:, 1:100) / 4;

[kpoldState, keypointsState, keypointsKltLostState, keep] = obj.klt.KltTracker(inputHandler, ii, keypointsState);
[kpoldNew, keypointsNew, keypointsKltLostNew, ~] = obj.klt.KltTracker(inputHandler, ii, keypointsNew);
keypointsKltLost = [keypointsKltLostState, keypointsKltLostNew] * 4;
keypoints = [keypointsState, keypointsNew] * 4;
kpold = [kpoldState, kpoldNew] * 4;
landmarksState = landmarksState(:, keep);
%     1. P3P using old landmarks
[R_CW, t_CW, inliers] = p3pRANSAC(keypointsState, landmarksState, K,...
    obj.p3pRANSACIt, obj.p3pTolerance, obj.minInliers, obj.adaptive,...
    obj.verbose);
% Sospetto che ritorni il t opposto rispetto a quello giusto
% t_CW = -R_CW.' * t_CW;
% R_CW = R_CW.';
            
%       2. Triangulation to generate new landmarks
poseKlt = reshape(pose, [3, 4]);
poseKltNew = [R_CW, t_CW];    %new pose
if ~isempty(R_CW) && ~isempty(t_CW)
    [keypoints, landmarks] = triangulationForKlt (K, kpold,...
        keypoints, poseKlt, poseKltNew, 4);

    image = inputHandler.getImage(ii);

    %Roba per errorMetrics
    trackedKeypoints = keypoints;
    trackedLandmarks = landmarks;

    pose = [R_CW(:); t_CW(:)];
    lostKeypoints = keypointsKltLost;
    landmarksIdx = landmarksIdx(keep);
else
    pose = [];
    trackedKeypoints = keypoints;
end