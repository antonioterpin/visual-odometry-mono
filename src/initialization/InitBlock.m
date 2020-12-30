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
        nSkip = 1
        nIt = 3
        adaptive = 0.95
        stopWithNPoints = 90
        errorThreshold = 1
        maxDistance = 200
        nCandidates = 100
        
        configurableProps = {'verbose', 'RANSACIt', 'adaptive', ...
            'nSkip', 'nIt', 'errorThreshold', 'maxDistance', ...
            'stopWithNPoints', 'nKeypoints', 'nCandidates'}
    end
    
    methods
        function [keypoints,landmarks,T_2W,secondIndex, candidates, prevFrameKeypoints] ...
                = run(obj, fromIndex, T_1W)
        % TODO DOCUMENT
        arguments
          obj InitBlock
          fromIndex uint32 = 1
          T_1W {isTransformationMatrix} = eye(3,4)
        end
        if size(T_1W,1) == 3
            T_1W = [T_1W; 0 0 0 1];
        end
        [keypoints,landmarks,T_2W, secondIndex, candidates, prevFrameKeypoints] = obj.run_(fromIndex, T_1W);
        end
    end
    
    methods (Abstract, Access = protected)
        [keypoints,landmarks,T_2W,secondIndex, candidates, prevFrameKeypoints] ...
            = run_(obj, fromIndex, T_1W)
    end
end

function isTransformationMatrix(T)
    if size(T,1) < 3 || size(T,1) > 4 || size(T,2) ~= 4
        eid = 'Size:notValid';
        msg = 'T must be a valid transformation matrix.';
        throwAsCaller(MException(eid,msg))
    end
end
