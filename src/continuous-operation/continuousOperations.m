function [ pose, state, lastKeypoints, lastDescriptors, sizeHistory, keypointsOutliers] =...
    continuousOperations(pose1, image1, image, K, state1, lastKeypoints,...
    lastDescriptors, sizeHistory)
    %CONTINUOUSOPERATIONS
    
    %state1 is [12xN] state matrix (to reshape)
    % TODO read this from config file!
    lambda = 3;
    newPointsTolerance = 1;
    p3pRANSACIt = 2000;
	p3pTolerance = 3;
    triangulationSample = 8;
    newPointsRANSACIt = 1000;
    detector = HarrisDetectorBlock({}); % TODO OOP: property!
    
    %% Unpack state
    keypoints1 = state1(1:2, :); % AT: [U;V]  AA: [V;U]
    landmarks1 = state1(3:5, :); % [X;Y;Z]
    
    %% Process previous image
    descriptors1 = detector.describeKeypoints(image1, keypoints1);
    
    %% Process current image
    [keypoints, descriptors] = detector.extractFeatures(image);
    
    %% Find matches in two images
    [matches, keypoints1Matched, keypointsMatched] = detector.getMatches(...
        descriptors1, descriptors, keypoints1, keypoints);
    landmarksMatched = landmarks1(:, matches(matches > 0) );
    keypointsMatched = keypointsMatched(1:2, :);
    
    %% Find R and t with P3P + Ransac
    [R_IC, t_IC] = p3pRANSAC(keypointsMatched(1:2, :)', landmarksMatched', K,...
        p3pRANSACIt, p3pTolerance);
    if(isempty(R_IC) || isempty(t_IC))
        state = [];
        pose = [];
        keypointsOutliers = [];
        return
    else
        pose = [R_IC(:); t_IC(:)];  %12x1 pose vector
    end
    
%     keypointsMatched = flipud(keypointsMatched);
%     keypoints = flipud(keypoints);
        state = [keypointsMatched; landmarksMatched];
    
    %% Find unmatched keypoints
    % Retrieves the Descriptors and Keypoints without a Landmark-match
    unMatchedIndices = ~ismember(keypoints', keypointsMatched','rows'); %get index of unmatched keypoints
    % [V;U]
    keypointsTri = keypoints(:,unMatchedIndices);   % unmatched keypoints
    descriptorsTri = descriptors(:,unMatchedIndices);
    %OSS We get unmatched keypoints because the matched ones already have
    %their correspondent landmark in the state1 vector
    
    %% Find new landmarks with triangulation
    if(~isempty(lastKeypoints) && ~isempty(lastDescriptors))    %robustness check
        [p1, p, ~] = initializeTriangulation(descriptorsTri,...
            lastDescriptors, keypointsTri, lastKeypoints);
        lastPose = reshape(pose1, 3, 4);
        newState = [];
        [newState] = triangulationRANSAC(newState, K,...
            p1, p, lastPose, triangulationSample, newPointsTolerance,...
            newPointsRANSACIt);
    else
        newState = [];
    end
    % Add new Landmarks
    state = [state, newState];  %OSS Every iteration we accumulate more keypoints and landmarks in state
    poseW =-R_IC'*t_IC; %world pose
    % Filter out points not in front
    inFront = R_IC(3,1:3)*(state(3:5,:)-poseW) > 0;
    state = state(:, inFront == 1);
    
    disp([num2str(size(state,2)) ' landmarks currently being tracked'])
    
    %% Save all unmatched keypoints and descriptors for next triangulations
    unmatchedPostTriIdx= ~ismember(keypoints', p(1:2, :)','rows'); %Indexes of keypoints unmatched after triangulation
    unmatchedKeypoints = keypoints(:,unmatchedPostTriIdx);
    
    unmatchedDescriptors = descriptors(:, unmatchedPostTriIdx);
    
    [sizeHistory, lastKeypoints, lastDescriptors] =...
    maintainTriangulationHistory (sizeHistory, lastKeypoints, lastDescriptors,...
    unmatchedKeypoints, unmatchedDescriptors);
    
    %% Get data for plots
    outliers = ~ismember(keypoints1', keypoints1Matched(1:2, :)', 'rows');
    keypointsOutliers = keypoints1(1:2, outliers);
end