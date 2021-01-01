classdef KLTCOBlock < COBlock
    %KLTCOBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        r_T = 20
        nIt = 50
        lambda = 10^-3;
    end
    
    methods
        function obj = KLTCOBlock()
            obj.configurableProps = [obj.configurableProps, ...
                'r_T', 'nIt', 'lambda'];
        end
    end
    
    methods %(Access = protected)
        function [trKp, kpMask] = track(obj, prevFrameIdx, frameIdx, kp1, tracker)
            verboseDisp(obj.verbose, 'KLT to localize');

            % Get images
            image1 = obj.inputBlock.getImage(prevFrameIdx);
            image2 = obj.inputBlock.getImage(frameIdx);
            
%             [trKp,kpMask] = KLT(image1,image2,kp1,obj.r_T,obj.nIt,obj.lambda);

            release(tracker);
            tracker = vision.PointTracker(...
                'MaxBidirectionalError',obj.lambda, ...
                'BlockSize', [obj.r_T, obj.r_T], ...
                'MaxIterations', obj.nIt, ...
                'NumPyramidLevels', 3);
%             tracker.setPoints(kp1.');
            tracker.initialize(kp1.', image1);
            [trKp,kpMask] = tracker(image2);
%             release(tracker);
            trKp = trKp(kpMask,:).';
        end
    end
end



