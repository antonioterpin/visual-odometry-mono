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
    
    if isempty(state)
        % Run from the beginning
        obj.state.addPose(obj.startingFrame, eye(3), zeros(3,1));
    else
        obj.state = state;
    end
    
    obj.outBlock.state = obj.state; % sync output with pipeline
    
    K = obj.inputBlock.getIntrinsics();
    
    prevFrameIdx = obj.startingFrame;
    obj.outBlock.addHistoryEntry(prevFrameIdx, []);
    nFrameProcessed = 0;
    while prevFrameIdx < min(obj.inputBlock.getNumberOfImages(), obj.lastFrame)
        
        % If we are tracking to fre keypoints, we re-initialize
        if obj.state.isLost()
            initialization;
        else
            frameIdx = prevFrameIdx + obj.nSkip;
            continuousOperation;
        end

        obj.state.isLost(~localized);
        if localized
            printMetrics;

            % Plot
            figure(1);
            obj.outBlock.plot(frameIdx, trackedKeypoints, trackedCandidates, trackedLandmarks)
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