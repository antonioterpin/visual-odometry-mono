classdef P3PRANSACCOBlock
    %COBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Detector %DetectorBlock = HarrisDetectorBlock({})
        K
        newPointsTolerance = 1
        p3pRANSACIt = 2000
        p3pTolerance = 3
        triangulationSample = 8
        newPointsRANSACIt = 2000
        minInliers = 30
        verbose = true
        
        % Data kept in memory between a localization and a triangulation
%         keypoints
    end
    
    properties (Constant)
        configurableProps = { 'newPointsTolerance', 'p3pRANSACIt', ...
            'verbose', 'minInliers', ...
            'p3pTolerance', 'triangulationSample', 'newPointsRANSACIt'}
    end
    
    methods 
        function [obj, keypointsMatched, descriptorsMatched, landmarksMatched, R_CW, t_CW, tracked_mask] ...
                = localize(obj, trackedDescriptors, trackedLandmarks, image)
            
            verboseDisp(obj.verbose, 'p3p + RANSAC to localize');
            
            [keypoints, descriptors] = obj.Detector.extractFeatures(image);

            matches = obj.Detector.getMatches(trackedDescriptors, descriptors);

            [~, matched_idx, trackedMatched_idx] = find(matches);

            landmarksMatched = trackedLandmarks(:, trackedMatched_idx);
            keypointsMatched = keypoints(1:2, matched_idx);
            descriptorsMatched = descriptors(:, matched_idx);
            
            if size(keypointsMatched, 2) >= obj.minInliers
               [R_CW, t_CW, inliers] = p3pRANSAC(...
                keypointsMatched, landmarksMatched, obj.K, obj.p3pRANSACIt, ...
                obj.p3pTolerance, obj.minInliers, obj.verbose); 
            else
                inliers = 0;
                R_CW = []; 
                t_CW = [];
            end
            
            tracked_mask = zeros(size(trackedDescriptors, 2), 1);
            if nnz(inliers) == 0
                keypointsMatched = [];
                landmarksMatched = [];
                descriptorsMatched = [];
            else
                keypointsMatched = keypointsMatched(:,inliers);
                landmarksMatched = landmarksMatched(:,inliers);
                descriptorsMatched = descriptorsMatched(:,inliers);
                
                tracked_mask(trackedMatched_idx(inliers)) = 1;
            end
            tracked_mask = tracked_mask > 0;
        end
    end
end

