classdef MonoVOPipeline
%MONOVOPIPELINE Summary of this class goes here
%   Detailed explanation goes here

% Config params
properties (Constant)
    configurableProps = {'verbose', 'nSkip', 'continuouslyTriangulate', ...
        'startingFrame', 'lastFrame', 'nFramesBeforePlot'}
end
properties
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
    continuouslyTriangulate = true
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
            initialization;
        else
            continuousOperation;
        end

        obj.state = obj.state.isLost(isempty(pose));
        if ~isempty(pose)
            R_CW = reshape(pose(1:9), 3, 3);
            t_CW = reshape(pose(10:12), 3, 1);
            
            printMetrics;
            
            % Update state
            obj.state = obj.state.addPose(ii, R_CW, t_CW);
            obj.state = obj.state.addLandmarksToPose(...
                ii, landmarksIdx, trackedKeypoints.');
            
            % Triangulation
            otherKeypoints = lostKeypoints;
            if ~isempty(unmatchedKeypoints) && obj.continuouslyTriangulate
                obj.state = triangulateNewData(obj.state, K, ...
                    obj.coBlock.Detector, obj.nSkip, ii, ...
                    unmatchedKeypoints, unmatchedDescriptors, obj.verbose);
                otherKeypoints = unmatchedKeypoints;
            end

            % Plot
            image = inputHandler.getImage(ii);
            positionHistory = obj.state.getPositions();
            landmarksHistory(end+1) = obj.state.getNumberOfLandmarksTrackedAtFrame(ii);
            figure(1);
            plotTogether(image, trackedKeypoints, otherKeypoints, ...
                positionHistory, trackedLandmarks, landmarksHistory, ...
                obj.nSkip, obj.startingFrame);
%             figure(2);
%             plot(obj.state.ObservationGraph)
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