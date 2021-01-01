classdef (Abstract) InitBlock < handle
    %INITIALIZATIONBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector
        inputBlock
        K
        
        nKeypoints = 2000
        verbose = false
        RANSACIt = 2000
        adaptive = 0.95
        frameOffset = 2
        errorThreshold = 1
        maxDistance = 200
        candidateSuppressionRadius = 10
        nLandmarksReference = 200
        samplingSize = [3,3]
        
        configurableProps = {'verbose', 'RANSACIt', 'adaptive', ...
            'errorThreshold', 'maxDistance', ...
            'nKeypoints', 'nLandmarksReference', 'frameOffset', ...
            'candidateSuppressionRadius', 'samplingSize'}
    end
    
    methods
        function [kp,P_W,T_2W,frameIdx,prevKp,kpc,tracker] = run(obj, fromIdx, T_1W)
        % TODO DOCUMENT
        arguments
          obj InitBlock
          fromIdx uint32 = 1
          T_1W {isTransformationMatrix} = eye(3,4)
        end
        if size(T_1W,1) == 3
            T_1W = [T_1W; 0 0 0 1];
        end
        
        frameIdx = fromIdx + obj.frameOffset;
        
        [kp,P_W,T_2W,prevKp,tracker] = obj.run_(fromIdx, frameIdx, T_1W);
        
        if ~isempty(kp)
            % Candidate new keypoints
            error = max(0, obj.nLandmarksReference - size(kp, 2));
            kpc = [];
            if error > 0
                image2 = obj.inputBlock.getImage(frameIdx);
                mask = obj.detector.getMask(size(image2), floor(kp),...
                    obj.candidateSuppressionRadius);

                newCandidates = repmat(...
                    floor(error / prod(obj.samplingSize)),...
                    reshape(obj.samplingSize, 1, 2));
                kpc = obj.detector.extractFeatures(...
                    image2, newCandidates, mask);
            end
        end
        
        end
    end
    
    methods (Abstract, Access = protected)
        [kp,P_W,T_2W,frameIdx,prevKp,tracker] = run_(obj, fromIndex, T_1W)
    end
end

function isTransformationMatrix(T)
    if size(T,1) < 3 || size(T,1) > 4 || size(T,2) ~= 4
        eid = 'Size:notValid';
        msg = 'T must be a valid transformation matrix.';
        throwAsCaller(MException(eid,msg))
    end
end
