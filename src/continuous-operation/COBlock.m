classdef (Abstract) COBlock < handle
    %COBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector
        inputBlock
        
        K
        p3pRANSACIt = 2000
        p3pTolerance = 3
        minInliers = 30
        adaptive = 0
        verbose = false
        candidateSuppressionRadius = 10
        nLandmarksReference = 200
        samplingSize = [3,3]
        keyframeConfidence = 0.8
        keyframeMaxSkip = 5
        
        configurableProps = { 'p3pRANSACIt', 'p3pTolerance', ...
            'verbose', 'minInliers', 'adaptive', 'nLandmarksReference', ...
            'candidateSuppressionRadius', 'samplingSize', ...
            'keyframeConfidence', 'keyframeMaxSkip'}
    end
    
    methods
        function [R_CW, t_CW, trKp, kpMask, trKpc, kpcMask, newKpc] = ...
                localize(obj, prevFrameIdx, frameIdx, kp, P_W, kpc, tracker)
            
            % 1. Keyframe selection & tracking
            nKp = size(kp, 2);
            [trKpKpc, kpkpcMask] = obj.track(prevFrameIdx, frameIdx, [kp, kpc], tracker);
            
            kpMask = kpkpcMask(1:nKp);
            kpcMask = kpkpcMask(nKp+1:end);
            
            trKp = trKpKpc(:, 1:nnz(kpMask));
            trKpc = trKpKpc(:, nnz(kpMask)+1:end);
            
            verboseDisp(obj.verbose, ...
                'Found %d matches and tracked %d candidates.\n', ...
                [size(trKp,2), size(trKpc,2)]);
            
            % 2. Localization
            if nnz(kpMask) >= obj.minInliers
               [R_CW, t_CW, inliers] = p3pRANSAC(...
                    trKp, P_W(:,kpMask), obj.K, ...
                    obj.p3pRANSACIt, obj.p3pTolerance, ...
                    obj.minInliers, obj.adaptive, obj.verbose); 
                
                % 3. Output
                kpMask(kpMask > 0) = inliers;
                trKp = trKp(:,inliers);
                
                % Candidate new keypoints
                error = max(0, obj.nLandmarksReference - size(trKpKpc, 2));
                newKpc = [];
                if error > 0
                    image2 = obj.inputBlock.getImage(frameIdx);
                    mask = obj.detector.getMask(...
                        size(image2), floor([trKp, trKpc]), obj.candidateSuppressionRadius);

                    newCandidates = repmat(...
                        floor(error / prod(obj.samplingSize)),...
                        reshape(obj.samplingSize, 1, 2));
                    newKpc = obj.detector.extractFeatures(...
                        image2, newCandidates, mask);
                end
            else
                kpMask = [];
                newKpc = [];
                trKp = [];
                R_CW = []; 
                t_CW = [];
                verboseDisp(obj.verbose, ...
                    'Too few matches to localize.\n', []);
            end
        end
    end
    
    methods (Abstract) %Access = protected
        [trKp, kpMask] = track(obj, prevFrameIdx, frameIdx, kp, tracker);
    end
end

