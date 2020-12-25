classdef MonoVOPipeline
    %MONOVOPIPELINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        inputBlock
        initBlock
        coBlock
        optBlock
        outBlock
        % VO Pipeline params
        verbose = true
        nSkip = 1
        startingFrame = 1
        lastFrame = Inf
        nFramesBeforePlot = 1
        lostBelow = 20
    end
    
    properties (Access = private, Constant)
        configurableProps = {'verbose', 'nSkip', ...
            'startingFrame', 'lastFrame', ...
            'nFramesBeforePlot', 'lostBelow'}
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
            
            configuredProps = obj.configurableProps(...
                isfield(configuration.PipelineParams, obj.configurableProps));
            for prop = configuredProps
                obj.(prop{:}) = configuration.PipelineParams.(prop{:});
            end
            
        end
        
        function run(obj)
            arguments
                obj MonoVOPipeline
%                 pipelineState PipelineState = PipelineState(obj.inputBlock.getIntrinsics());
            end
            
            inputHandler = obj.inputBlock;
            K = obj.inputBlock.getIntrinsics();
            
            pose = reshape(eye(3,4), 1, []);
            state = [];
            
            % Continuous operation
            positionHistory = zeros(3,1);
            poseHistory = reshape(pose, [], 1);
            landmarksHistory = [];

            ii = obj.startingFrame + obj.nSkip;
            while ii <= min(inputHandler.getNumberOfImages(), obj.lastFrame)
                % If we are tracking to fre keypoints, we re-initialize
                if isempty(pose) || size(state, 2) < obj.lostBelow
                    verboseDisp(obj.verbose, ...
                        '\n\nInit from frame %d\n=====================\n', ii - obj.nSkip);
                    
                    prevPose = reshape(poseHistory(:,end), 3, 4);

                    [trackedKeypoints, trackedLandmarks, trackedDescriptors, pose, ii] = ...
                        obj.initBlock.run(inputHandler, K, ii-obj.nSkip, prevPose);
                    
                    lostKeypoints = [];
                    if ~isempty(pose)
                        pose = reshape(pose(1:3,:), [], 1);
                        trackedLandmarks = trackedLandmarks(1:3,:);
                    else
                        poseHistory = poseHistory(:, 1:end - 1);
                        landmarksHistory = landmarksHistory(1:end-1);
                        ii = ii - obj.nSkip;
                    end
                else
                    verboseDisp(obj.verbose, ...
                        '\n\nProcessing frame %d\n=====================\n', ii);
                    % Unpack state
                    keypoints = state(1:2,:);
                    landmarks = state(3:5,:);
%                     descriptors = state(6:end,:);

                    prevImage = inputHandler.getImage(ii - obj.nSkip);
                    descriptors = obj.coBlock.Detector.describeKeypoints(prevImage, keypoints);
                    
                    image = inputHandler.getImage(ii);
                    
                    [obj.coBlock, trackedKeypoints, trackedDescriptors, trackedLandmarks, R_CW, t_CW, lost_idx] ...
                        = obj.coBlock.localize(descriptors, landmarks, image);
                    
                    if isempty(R_CW) || isempty(t_CW)
                        pose = [];
                        ii = ii - obj.nSkip; % this frame has to be repeated
                    else
                        pose = [R_CW(:); t_CW(:)];
                        lostKeypoints = keypoints(:, lost_idx);
%                         [validLandmarks, ~] = filterPoints(trackedLandmarks, R_CW, t_CW);
%                         trackedKeypoints = trackedKeypoints(:,validLandmarks);
%                         trackedDescriptors = trackedDescriptors(:,validLandmarks);
%                         trackedLandmarks = trackedLandmarks(:,validLandmarks); 
                    end
                end
                
                if ~isempty(pose)
                    % Filter points
                    poseHistory = [poseHistory, pose];
                    R_CW = reshape(pose(1:9), 3, 3);
                    t_CW = reshape(pose(10:12), 3, 1);
                    
                    % Evaluate reprojection error
                    points_C = R_CW * trackedLandmarks ...
                        + repmat(t_CW, [1, size(trackedLandmarks, 2)]);
                    repr = projectPoints(points_C, K);
                    repr_err = sum((trackedKeypoints - repr).^2, 'all');
                    N = size(trackedLandmarks, 2);
                    fprintf(strcat(...
                    '=====================\n', ...
                    'Reprojection error on tracked landmarks (%d)\n',...
                    '    Tot: %.3f\n', ...
                    '    Avg: %.3f\n',...
                    '=====================\n'), N, repr_err, repr_err / N);

                    state = [double(trackedKeypoints);
                        trackedLandmarks;
                        double(trackedDescriptors)];
                    
                    % Add pose to history
                    positionHistory = [positionHistory, -R_CW.' * t_CW];
                
                    image = inputHandler.getImage(ii);
                    landmarksHistory = [landmarksHistory; N];
                    plotTogether(image, trackedKeypoints, lostKeypoints, ...
                        positionHistory, trackedLandmarks, landmarksHistory, ...
                        obj.nSkip, obj.startingFrame);
                end
                  
                ii = ii + obj.nSkip;
                pause(0.01); % Makes sure that plots refresh.  
            end
        end
    end
end