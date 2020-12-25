classdef (Abstract) InitBlock
    %INITIALIZATIONBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Detector %DetectorBlock = HarrisDetectorBlock({})
    end
    
    methods
        function [keypoints,landmarks,descriptors,T_2W,secondIndex] = ...
              run(obj, input, K, fromIndex, T_1W)
        % TODO DOCUMENT
        arguments
          obj InitBlock
          input InputBlock
          K (3,3)
          fromIndex uint32 = 1
          T_1W {isTransformationMatrix} = eye(3,4)
        end
        if size(T_1W,1) == 3
            T_1W = [T_1W; 0 0 0 1];
        end
        [keypoints,landmarks,descriptors,T_2W,secondIndex] = ...
            obj.run_(input, K, fromIndex, T_1W);
        end
    end
    
    methods (Abstract, Access = protected)
        [keypoints,landmarks,descriptors,T_2W,secondIndex] = ...
            run_(obj, input, K, fromIndex, T_1W)
    end
end

function isTransformationMatrix(T)
    if size(T,1) < 3 || size(T,1) > 4 || size(T,2) ~= 4
        eid = 'Size:notValid';
        msg = 'T must be a valid transformation matrix.';
        throwAsCaller(MException(eid,msg))
    end
end
