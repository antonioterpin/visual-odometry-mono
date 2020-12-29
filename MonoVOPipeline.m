classdef MonoVOPipeline < handle
%MONOVOPIPELINE Summary of this class goes here
%   Detailed explanation goes here

% Config params
properties (Constant)
    configurableProps = {'verbose', 'nSkip', 'continuouslyTriangulate', ...
        'startingFrame', 'lastFrame'}
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
    
    %KLT IMPLEMENTATION     TODO change this part
    useKlt = true;  %TODO add to json
    klt = Klt;
    p3pRANSACIt = 2000
    p3pTolerance = 1
    triangulationSample = 8
    newPointsRANSACIt = 2000
    minInliers = 5
    adaptive = 0.99
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
        obj.state.addPose(obj.startingFrame, eye(3), zeros(3,1));
    else
        obj.state = state;
    end

    landmarksHistory = [];
    keypointsToAdd = [];    %TODO remove this from here
    
    obj.outBlock.state = obj.state; % sync output with pipeline
    
    prevFrameIdx = obj.startingFrame;
    obj.outBlock.addHistoryEntry(prevFrameIdx, []);
    nFrameProcessed = 0;
    while prevFrameIdx < min(inputHandler.getNumberOfImages(), obj.lastFrame)
        
        % If we are tracking to fre keypoints, we re-initialize
        if obj.state.isLost()
            initialization;
        else
            frameIdx = prevFrameIdx + obj.nSkip;
            if obj.useKlt   %TODO remove this (klt)
                kltBlock;
            else
                continuousOperation;
            end
        end

        obj.state.isLost(isempty(pose));
        if ~isempty(pose)
            R_CW = reshape(pose(1:9), 3, 3);
            t_CW = reshape(pose(10:12), 3, 1);
            
            printMetrics;
            
            % Update state
            obj.state.addPose(frameIdx, R_CW, t_CW);
            obj.state.addLandmarksToPose(frameIdx, landmarksIdx, trackedKeypoints.');

            % Triangulation
            otherKeypoints = lostKeypoints;
%             if ~isempty(unmatchedKeypoints) && obj.continuouslyTriangulate
%                 triangulateNewData(obj.state, K, ...
%                     obj.coBlock.Detector, obj.nSkip, frameIdx, ...
%                     unmatchedKeypoints, unmatchedDescriptors, obj.verbose);
%                 otherKeypoints = unmatchedKeypoints;
%             end

            % Plot
            figure(1);
            obj.outBlock.plot(frameIdx, trackedKeypoints, otherKeypoints, trackedLandmarks)
%             figure(2);
%             plot(obj.state.ObservationGraph)
            
            nFrameProcessed = nFrameProcessed + 1;
            prevFrameIdx = frameIdx;
            
            % Eventually bundle adjust
            if obj.optBlock.isActive ...
                    && mod(nFrameProcessed, obj.optBlock.everyNIterations) == 0
                % Get optimization data structure
                [hiddenState, observations, bundleIdx] ...
                    = obj.state.getOptimizationDS(obj.optBlock.maxBundleSize);
                
                % Start optimization
                hiddenState = obj.optBlock.optimize(hiddenState, observations);
                
                % Update state
                obj.state.optimizedBundle(hiddenState, bundleIdx);
            end
        end
        
        pause(0.01); % Makes sure that plots refresh.  
    end
end
end
end