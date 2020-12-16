classdef (Abstract) InitBlock
    %INITIALIZATIONBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Detector DetectorBlock = HarrisDetectorBlock({})
    end
    
    methods
        function [keypoints,landmarks,descriptors,secondIndex] = ...
              run(obj, input, K, fromIndex, stepIndex, nIt, T_1W)
        arguments
          obj InitBlock
          input InputBlock
          K (3,3)
          fromIndex uint32 = 1
          stepIndex uint32 = 1
          nIt uint32 = 1
          T_1W {isTransformationMatrix} = eye(3,4)
        end
        if size(T_1W,1) == 3
            T_1W = [T_1W; 0 0 0 1];
        end
        [keypoints,landmarks,descriptors,secondIndex] = ...
            obj.run_(input, K, fromIndex, stepIndex, nIt, T_1W);
        end
    end
    
    methods (Abstract)
        obj = setParams(obj, params);
    end
    
    methods (Abstract, Access = protected)
        [keypoints,landmarks,descriptors,secondIndex] = ...
            run_(obj, input, K, fromIndex, stepIndex, nIt, T_1W)
    end
end

function isTransformationMatrix(T)
    if size(T,1) < 3 || size(T,1) > 4 || size(T,2) ~= 4
        eid = 'Size:notValid';
        msg = 'T must be a valid transformation matrix.';
        throwAsCaller(MException(eid,msg))
    end
end