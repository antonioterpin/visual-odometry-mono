classdef PatchMatchingInitBlock < InitBlock
    %PATCHMATCHINGINITBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        verbose = false
        RANSACIt = 2000
    end
    
    methods 
        function obj = setParams(obj, params)
            % TODO !!!
        end
    end
    
    methods (Access = protected)
        function [keypoints,landmarks,descriptors,T_2W,secondIndex] = ...
            run_(obj, input, K, fromIndex, stepIndex, nIt, T_1W)
            
            verboseDisp(obj.verbose, 'Bootstrapping...');
            
            image1 = input.getImage(fromIndex);
            [keypoints1, descriptors1] ...
                = obj.Detector.extractFeatures(image1);
            
            
            % Search best frame to start looking at epipolar line distance
            R_1W = T_1W(1:3,1:3);
            t_1W = T_1W(1:3,4);
            maxInliersCount = -1;
            for it = fromIndex+stepIndex:stepIndex:fromIndex+stepIndex*nIt
                verboseDisp(obj.verbose, ...
                    'Evaluating frame %d for bootstrapping\n', it);
                
                image2 = input.getImage(it);
                [keypoints2, descriptors2] ...
                    = obj.Detector.extractFeatures(image2);
                [matchesIdx, p1_, p2_] = obj.Detector.getMatches(...
                    descriptors1, descriptors2, keypoints1, keypoints2);
                
                if size(p1_, 2) < 8 % Minimum number of parameters
                    verboseDisp(obj.verbose, ...
                        'Skipping frame %d due to too few matches (%d)', ...
                        frameIndex, size(p0_candidate, 2));
                    continue;
                end
                
                p1 = [p1_; ones(1,size(p1_,2))];
                p2 = [p2_; ones(1,size(p2_,2))];
                F_candidate = estimateFundamentalMatrix(p1,p2,true);
                inliercount = nnz(errorMetric(p1,p2,F_candidate(:)));

                if inliercount > maxInliersCount
                    maxInliersCount = inliercount;
                    best_p1 = p1;
                    best_p2 = p2;
                    secondIndex = it;
                    descriptors = descriptors2(:,matchesIdx > 0);
                end
            end
            
            assert(maxInliersCount > 0, 'Could not find inliers.');
            
            verboseDisp(obj.verbose, ...
                'Initializing with frame %s', num2str(secondIndex));
            p1 = best_p1;
            p2 = best_p2;
            
            % RANSAC
            [~, inliers] = RANSAC(...
                @(data, otherParams) ...
                    estimateFundamentalMatrix(data(1:3,:), data(4:6,:), true), ...
                @(candidate, data, otherParams) ...
                    errorMetric(data(1:3,:), data(4:6,:), candidate(:)),...
                8, [p1; p2], obj.RANSACIt, [], obj.verbose);

            p1 = p1(:,inliers);
            p2 = p2(:,inliers);
            descriptors = descriptors(:,inliers);
            
            % Estimate pose from inliers
            T_21 = estimateRelativePose(p1, p2, K, K);
            landmarks = triangulateFromPose(p1, p2, T_21, K, K, T_1W);

            [validLandmarks, landmarks] = filterPoints(landmarks, R_1W, t_1W);

            keypoints = p2(1:2,validLandmarks);
            descriptors = descriptors(:,validLandmarks);
            landmarks = landmarks(1:3,:);
            T_2W = T_21(1:3,:) * T_1W;
        end
    end
end

