classdef MonoVOPipeline
    %MONOVOPIPELINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        inputBlock
        initBlock
        coBlock
        optBlock
        outBlock
    end
    
    methods
        function obj = MonoVOPipeline(configuration)
            %MONOVOPIPELINE Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                configuration Config
            end
            
            obj.inputBlock = configuration.InputHandler;
            obj.initBlock = configuration.InitializationHandler;
            obj.coBlock = configuration.ContinuousOperationHandler;
            obj.optBlock = configuration.OptimizationHandler;
            obj.outBlock = configuration.OutputHandler;
        end
        
        function run(obj)
            K = obj.inputBlock.getIntrinsics();
            inputHandler = obj.inputBlock;
            
            prevImage = inputHandler.getImage(1);
            
            fprintf('\n\nProcessing frame %d\n=====================\n', 1);
[firstKeypoints,firstLandmarks,bestDescriptors,nStart] =...
    obj.initBlock.run(inputHandler, K, 1, 2, 2, eye(3,4));
prevState = [firstKeypoints;firstLandmarks(1:3,:)];
%% Continuous operation
I = eye(3);
I = I(:);
poseHistory = [0;0;0];
landmarksHistory = [];
currState = prevState;
lastKeypoints = firstKeypoints;
lastDescriptors = bestDescriptors;
sizeHistory = size(firstKeypoints, 2);

nSkip = 2;  %number of skipped frames every iteration
% nStart = 5;
for ii = nStart+1: nSkip :inputHandler.getNumberOfImages()
    fprintf('\n\nProcessing frame %d\n=====================\n', ii);
     currImage = inputHandler.getImage(ii);
    
    if ii == nStart + 1 
        currPose = [I; 0;0;0];  %Initialize pose
        prevPose = currPose;
    elseif(~isempty(currPose))
        prevPose = reshape(currPose, 3, 4);    %Maintain last pose in case we get lost
    end

    %check to see if we're close and if so, re initialize
    if(isempty(currPose) || size(currState, 2) < 8)
        disp('Lost, will have to reinitialize from last pose')

        [firstKeypoints, firstLandmarks, descriptors] = ...
            obj.initBlock.run(inputHandler, K, ii-2, 2, 1, prevPose);
        lastDescriptors = [lastDescriptors, descriptors];
        lastKeypoints = [lastKeypoints, firstKeypoints];
        currState = [firstKeypoints;firstLandmarks(1:3,:)];
        currPose = prevPose(:); %TODO Change this with actual pose output from monoInitialization
        
        [sizeHistory, lastKeypoints, lastDescriptors] =...
        maintainTriangulationHistory (sizeHistory, lastKeypoints, lastDescriptors,...
        firstKeypoints, descriptors);
        
    else
        [currPose, currState, lastKeypoints, lastDescriptors, sizeHistory, outliers] = ...
        continuousOperations(currPose, prevImage, currImage, K, prevState, ...
        lastKeypoints, lastDescriptors, sizeHistory);
        if outliers == -1
            outliers = [];
        end
    end
    if(~isempty(currPose))  %TODO controlla che questo non lasci indietro la traiettoria rispetto al current status
        R_IC = reshape(currPose(1:9), 3,3);
        t_IC = reshape(currPose(10:12), 3, 1);
        poseHistory = [poseHistory, -R_IC.' * t_IC];
        inliers = currState(1:2, :)';
        %outliers = [];  %TODO add outliers
        landmarks = currState(3:5, :)';
            keypoints = currState(1:2, :)';
            %outliers
%            keypoints1
%            matchesInliers
        landmarksHistory = [landmarksHistory, size(landmarks, 1)];
        if(ii > 6)
            plotTogether(currImage, inliers, outliers, poseHistory, landmarks,...
                landmarksHistory, nSkip, nStart)
        end
    end
    
    prevState = currState;
    prevImage = currImage;
    % Makes sure that plots refresh.    
    pause(0.001);
    
end
        end
    end
end

