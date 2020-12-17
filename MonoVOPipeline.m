classdef MonoVOPipeline
    %MONOVOPIPELINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        inputBlock
        initBlock
        coBlock
        optBlock
        outBlock
        verbose
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
            obj.verbose = true; % TODO from config!
        end
        
        function run(obj, pipelineState)
            arguments
                obj MonoVOPipeline
                pipelineState PipelineState = PipelineState(obj.inputBlock.getIntrinsics());
            end
            
            % Params!!
            plotAfterNFrames = 3;
            nSkip = 2;
            global detector p3pRANSACIt p3pTolerance detector;
            detector = HarrisDetectorBlock({}); % TODO OOP: property!
            
            randomParamsThatHaveToBeFixed();
            
            inputHandler = obj.inputBlock;
            K = obj.inputBlock.getIntrinsics();
            
            prevImage = inputHandler.getImage(1);
            
            verboseDisp(obj.verbose, '\n\nProcessing frame %d\n=====================\n', 1);
            [lastKeypoints,landmarks,descriptors,currPose,nStart] =...
                obj.initBlock.run(inputHandler, K, 1, 2, 2, eye(3,4));
            
            currPose = reshape(currPose(1:3,:),[],1);
            prevPose = currPose;
            
            % Init state
            pipelineState = pipelineState.addLandmarks(currPose, ...
                landmarks.', lastKeypoints.', descriptors.');
            currState = [lastKeypoints; landmarks];
            prevState = currState;
            
            % Continuous operation
            poseHistory = zeros(3,1);
            landmarksHistory = [];
            sizeHistory = size(lastKeypoints, 2);
            triangulationCandidatesKeypoints = [];
            triangulationCandidatesDescriptors = [];

            for ii = nStart+1: nSkip :inputHandler.getNumberOfImages()
                verboseDisp(obj.verbose, '\n\nProcessing frame %d\n=====================\n', ii);
                 currImage = inputHandler.getImage(ii);

                if(~isempty(currPose)) && ii > nStart + 1
                    prevPose = reshape(currPose, 3, 4);    %Maintain last pose in case we get lost
                end

                %check to see if we're close and if so, re initialize
                if(isempty(currPose) || size(currState, 2) < 8)
                    verboseDisp(obj.verbose, 'Lost, will have to reinitialize from last pose');

                    % RE-INIT HERE
                    [nextKeypoints, nextLandmarks, nextDescriptors, currPose] = ...
                        obj.initBlock.run(inputHandler, K, ii-2, 2, 1, prevPose);
                    currPose = reshape(currPose(1:3,:), [], 1);
                    nextLandmarks = nextLandmarks(1:3,:);
                else
                    % Unpack state
                    keypoints1 = prevState(1:2,:);
                    landmarks1 = prevState(3:5,:);
                    
                    % Find matches in two images
                    descriptors1 = detector.describeKeypoints(prevImage, prevState(1:2,:));
                    [keypoints, descriptors] = detector.extractFeatures(currImage);
    
                    [matches, keypoints1Matched, keypointsMatched] = detector.getMatches(...
                        descriptors1, descriptors, keypoints1, keypoints);
                    landmarksMatched = landmarks1(:, matches(matches > 0) );
                    keypointsMatched = keypointsMatched(1:2, :);
                    descriptorsMatched = descriptors(:,matches > 0);
                    
                    [R_IC, t_IC] = p3pRANSAC(...
                        keypointsMatched(1:2, :)', landmarksMatched', ...
                        K, p3pRANSACIt, p3pTolerance);
                    currPose = [R_IC(:); t_IC(:)];  %12x1 pose vector
                    
                    outliers = keypoints1(:, ~ismember(keypoints1.', keypoints1Matched.', 'rows'));
                    
                    if ~isempty(currPose)
                    [triangulatedKeypoints, triangulatedLandmarks, triangulatedDescriptors, ...
                            triangulationCandidatesKeypoints, triangulationCandidatesDescriptors, ...
                            sizeHistory] = triangulateNewData(K, keypoints, descriptors, keypointsMatched, ...
        triangulationCandidatesKeypoints, triangulationCandidatesDescriptors, sizeHistory, currPose);
                    
                    nextKeypoints = [keypointsMatched, triangulatedKeypoints];
                    nextLandmarks = [landmarksMatched, triangulatedLandmarks];
                    nextDescriptors = [descriptorsMatched, triangulatedDescriptors];
                    end
                end
                
                if ~isempty(currPose)
                % Filter points
                R_WC = reshape(currPose(1:9),3,3);
                t_WC = reshape(currPose(10:12),3,1);
                t_CW = -R_WC.' * t_WC;
                % TODO THERE MIGHT BE AN ERROR IN R_WC, t_CW, filter
                % points... think better !
                [validLandmarks, nextLandmarks] = filterPoints(nextLandmarks, R_WC, t_CW);
                nextKeypoints = nextKeypoints(:,validLandmarks);
                nextDescriptors = nextDescriptors(:,validLandmarks);
                
                % Update state
                pipelineState = pipelineState.addLandmarks(currPose, ...
                    nextLandmarks.', nextKeypoints.', nextDescriptors.');
                currState = [nextKeypoints;nextLandmarks];
                % Add pose to history
                poseHistory = [poseHistory, t_CW];
                
                    if ii > plotAfterNFrames * nSkip + nStart
                        % TODO keyframe selection basato su traiettoria pregressa
                        landmarksHistory = pipelineState.getNLandmarksHistory();
                        plotTogether(currImage, nextKeypoints, outliers, ...
                            poseHistory, landmarks, landmarksHistory, nSkip, nStart)
                    end
                end

                prevState = currState;
                prevImage = currImage;
                  
                pause(0.01); % Makes sure that plots refresh.  
            end
        end
    end
end

function randomParamsThatHaveToBeFixed()
%   TODO THESE HAVE TO BECOME CONTINUOUS OPERATIONS PROPERTIES!
global newPointsTolerance p3pRANSACIt p3pTolerance triangulationSample newPointsRANSACIt;
newPointsTolerance = 1;
p3pRANSACIt = 2000;
p3pTolerance = 3;
triangulationSample = 8;
newPointsRANSACIt = 1000;
end
