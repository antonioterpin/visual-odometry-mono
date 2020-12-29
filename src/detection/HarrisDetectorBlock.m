classdef HarrisDetectorBlock < DetectorBlock
    %HARRISDETECTORBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        patchSize = 11
        kappa = 0.08
        nKeypoints = 2000
        nonMaximaSuppressionRadius = 5
        descriptorRadius = 11
        lambda = 3
    end
    
    methods
        function obj = HarrisDetectorBlock()
            obj.configurableProps = [obj.configurableProps, ...
                'patchSize', 'kappa', 'nKeypoints', ...
                'lambda', 'nonMaximaSuppressionRadius', 'descriptorRadius'};
        end
    end
    
    methods (Access = protected)
        function [matchesIndices, p1, p2] = getMatches_(obj, ...
                descriptors1, descriptors2, keypoints1, keypoints2)
            
            matchesIndices = SSDMatchDescriptors(...
                descriptors2, descriptors1, obj.lambda);

            p1 = [];
            p2 = [];

            if ~isempty(keypoints1) && ~isempty(keypoints2)
                [~, keypoints2_idx, keypoints1_idx] = find(matchesIndices);

                keypoints1 = keypoints1(:, keypoints1_idx);
                keypoints2 = keypoints2(:, keypoints2_idx);

                p1 = keypoints1;
                p2 = keypoints2;
            end

        end
        
        function [keypoints, descriptors] = extractFeatures_(obj,image)
            scores = harrisScore(image, obj.patchSize, obj.kappa);
            keypoints = harrisSelectKeypoints(...
                scores, obj.nKeypoints, obj.nonMaximaSuppressionRadius);
            descriptors = obj.describeKeypoints_(image, keypoints);
        end
        
        function [descriptors] = describeKeypoints_(obj,image,keypoints)
            descriptors = harrisDescribeKeypoints(...
                image, keypoints, obj.descriptorRadius);
        end
    end
end

