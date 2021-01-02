classdef PatchMatchingInitBlock < InitBlock
    %PATCHMATCHINGINITBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Access = protected)
        
        function [kp,P_W,T_2W,prevKp,tracker] = run_(obj, idx1, idx2, T_1W)
            
            kp = []; P_W = []; T_2W = []; prevKp = [];
            
            verboseDisp(obj.verbose, 'Bootstrapping with PatchMatching...');
            tracker = [];
            
            image1 = obj.inputBlock.getImage(idx1);
            keypoints1 = obj.detector.extractFeatures(image1, obj.nKeypoints);
            descriptors1 = obj.detector.describeKeypoints(image1,keypoints1);
            
            image2 = obj.inputBlock.getImage(idx2);
            keypoints2 = obj.detector.extractFeatures(image2, obj.nKeypoints);
            descriptors2 = obj.detector.describeKeypoints(image2,keypoints2);
                
            [~, p1_, p2_] = obj.detector.getMatches(...
                descriptors1, descriptors2, keypoints1, keypoints2);
                
            if size(p1_, 2) < 8 % Minimum number of parameters
                verboseDisp(obj.verbose, 'Unable to localize.', []);
                return;
            end
            
            errorTh = obj.errorThreshold^2;
            otherParams = [obj.K(:); T_1W(:); errorTh; obj.maxDistance];
                
            p1 = [p1_; ones(1,size(p1_,2))];
            p2 = [p2_; ones(1,size(p2_,2))];

            [model, inliers] = RANSAC(...
                @relativePoseFromSample, @reprojectionErrorFromModel, ...
                8, [p1; p2], obj.RANSACIt, otherParams, true, obj.adaptive);

            T_21 = model;
            T_2W = T_21(1:3,:) * T_1W;
            kp = p2(1:2, inliers);
            prevKp = p1(1:2, inliers);

            % triangulation of valid landmarks (already filtered)
            P_W = triangulateFromPose(p1(:, inliers), p2(:, inliers), ...
                T_21, obj.K, obj.K, T_1W);
        end
    end
end