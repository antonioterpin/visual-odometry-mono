classdef PatchMatchingCOBlock < COBlock
    %PATCHMATCHINGCOBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nKeypoints = 2000;
    end
    
    methods
        function obj = PatchMatchingCOBlock()
            obj.configurableProps = [obj.configurableProps, ...
                'nKeypoints'];
        end
    end
    
    methods (Access = protected)
        function [trKp, kpMask] = track(obj, prevFrameIdx, frameIdx, kp1, ~)
            verboseDisp(obj.verbose, 'Patch matching to localize');
            
            prevI = obj.inputBlock.getImage(prevFrameIdx);
            descriptors1 = obj.detector.describeKeypoints(prevI, kp1);
            
            I = obj.inputBlock.getImage(frameIdx);
            trKp = obj.detector.extractFeatures(I, obj.nKeypoints);
            descriptors2 = obj.detector.describeKeypoints(I, trKp);
            
            matches = obj.detector.getMatches(descriptors2, descriptors1);
            kpMask = matches > 0;
            
            trKpIdx = matches(matches > 0);
            trKp = trKp(:,trKpIdx);
        end
    end
end

