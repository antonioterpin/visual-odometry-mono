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
        function [trKp, kpMask] = track(obj, ~, frameIdx, kp1, tracker)
            verboseDisp(obj.verbose, 'KLT to localize');

            % Get images
            image2 = obj.inputBlock.getImage(frameIdx);
            tracker.setPoints(kp1.');

            [trKp,kpMask] = tracker(image2);
            trKp = trKp(kpMask,:).';
        end
    end
end



