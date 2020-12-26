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

% Config params
properties (Constant)
    configurableProps = {'verbose', 'nSkip', ...
        'startingFrame', 'lastFrame', 'nFramesBeforePlot'}
end
properties
    % VO Pipeline params
    verbose = true
    nSkip = 1
    startingFrame = 1
    lastFrame = Inf
    nFramesBeforePlot = 1
    state PipelineState = PipelineState();
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

%     configuredProps = obj.configurableProps(...
%         isfield(configuration.PipelineParams, obj.configurableProps));
%     for prop = configuredProps
%         obj.(prop{:}) = configuration.PipelineParams.(prop{:});
%     end
    obj= Config.setParams(obj, configuration.PipelineParams);
    if isfield(configuration.PipelineParams, 'State')
        obj.state = Config.setParams(obj.state, configuration.PipelineParams.State);
    end
end

function run(obj, state)
    arguments
        obj MonoVOPipeline
        state = [] % Optional, to resume pipeline run, with starting frame
    end

    inputHandler = obj.inputBlock;
    K = obj.inputBlock.getIntrinsics();
    
    if isempty(state)
        % Run from the beginning
        obj.state = obj.state.addPose(...
            obj.startingFrame, eye(3), zeros(3,1));
    else
        obj.state = state;
    end

    % Continuous operation
    landmarksHistory = [];

    ii = obj.startingFrame + obj.nSkip;
    while ii <= min(inputHandler.getNumberOfImages(), obj.lastFrame)
        % If we are tracking to fre keypoints, we re-initialize
        [~, isLost] = obj.state.isLost();
        if isLost
            verboseDisp(obj.verbose, ...
                '\n\nInit from frame %d\n=====================\n', ii - obj.nSkip);

            [R_CW, t_CW] = obj.state.getLastPose();
            prevPose = [R_CW, t_CW];

            [trackedKeypoints, trackedLandmarks, ~, pose, ii] = ...
                obj.initBlock.run(inputHandler, K, ii-obj.nSkip, prevPose);

            lostKeypoints = [];
            if ~isempty(pose)
                pose = reshape(pose(1:3,:), [], 1);
                trackedLandmarks = trackedLandmarks(1:3,:);
                
                % TODO add observation also to previous frame
                [obj.state, trackedLandmarks, landmarksIdx, mask] ...
                    = obj.state.addLandmarks(trackedLandmarks);
                trackedKeypoints = trackedKeypoints(:, mask);
            else
                poseHistory = poseHistory(:, 1:end - 1);
                landmarksHistory = landmarksHistory(1:end-1);
            end
        else
            verboseDisp(obj.verbose, ...
                '\n\nProcessing frame %d\n=====================\n', ii);
            
            [landmarks, landmarksIdx, keypoints] ...
                = obj.state.getObservations(ii - obj.nSkip);

            prevImage = inputHandler.getImage(ii - obj.nSkip);
            descriptors = obj.coBlock.Detector.describeKeypoints(prevImage, keypoints);

            image = inputHandler.getImage(ii);

            [obj.coBlock, trackedKeypoints, ~, trackedLandmarks, R_CW, t_CW, tracked_mask] ...
                = obj.coBlock.localize(descriptors, landmarks, image);

            if isempty(R_CW) || isempty(t_CW)
                pose = [];
            else
                pose = [R_CW(:); t_CW(:)];
                lostKeypoints = keypoints(:, ~tracked_mask);
%                         [validLandmarks, ~] = filterPoints(trackedLandmarks, R_CW, t_CW);
%                         trackedKeypoints = trackedKeypoints(:,validLandmarks);
%                         trackedDescriptors = trackedDescriptors(:,validLandmarks);
%                         trackedLandmarks = trackedLandmarks(:,validLandmarks); 
                landmarksIdx = landmarksIdx(tracked_mask);
            end
        end

        obj.state = obj.state.isLost(isempty(pose));
        if ~isempty(pose)
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

            
            % Update state
            obj.state = obj.state.addPose(ii, R_CW, t_CW);
            obj.state = obj.state.addLandmarksToPose(...
                ii, landmarksIdx, trackedKeypoints.');

            % Plot
            image = inputHandler.getImage(ii);
            landmarksHistory = [landmarksHistory; N];
            positionHistory = obj.state.getPositions();
            
            plotTogether(image, trackedKeypoints, lostKeypoints, ...
                positionHistory, trackedLandmarks, landmarksHistory, ...
                obj.nSkip, obj.startingFrame);
        else
            ii = ii - obj.nSkip; % this frame has to be repeated
            obj.state = obj.state.resetToPose(ii);
        end

        ii = ii + obj.nSkip;
        pause(0.01); % Makes sure that plots refresh.  
    end
end
end
end