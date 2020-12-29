classdef KLTCOBlock < COBlock
    %KLTCOBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        r_T = 20
        nIt = 50
        lambda = 10^-3;
        candidateSuppressionRadius = 10;
    end
    
    methods
        function obj = KLTCOBlock()
            obj.configurableProps = [obj.configurableProps, ...
                'r_T', 'nIt', 'lambda', 'candidateSuppressionRadius'];
        end
    end
    
    methods %(Access = protected)
        function [trKp, kpMask, newKpc] = track(obj, prevFrameIdx, frameIdx, kp1)
            verboseDisp(obj.verbose, 'KLT to localize');

            % Get images
            image1 = obj.inputBlock.getImage(prevFrameIdx);
            image2 = obj.inputBlock.getImage(frameIdx);
            
            % Sampling
            image1_ = imresize(image1, 0.25); kp1 = kp1 / 4;
            image2_ = imresize(image2, 0.25);

            dkp = zeros(size(kp1));
            kpMask = true(1, size(kp1, 2));
            r_T = obj.r_T; nIt = obj.nIt; lambda = obj.lambda; % parfor efficiency
            parfor j = 1:size(kp1, 2)
                [dkp(:,j), kpMask(j)] = trackKLTRobustly(image1_, image2_, ...
                    kp1(:,j).', r_T, nIt, lambda);
            end
            kpMask = kpMask > 0;
            trKp = kp1(:,kpMask) + dkp(:,kpMask);

            % Interpolation
            trKp = trKp * 4;
            
            % Candidate new keypoints
            mask = obj.detector.getMask(...
                size(image2), floor(trKp), obj.candidateSuppressionRadius);
            newKpc = obj.detector.extractFeatures(image2, mask);
        end
    end
end



