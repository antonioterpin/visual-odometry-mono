classdef PatchMatchingInitBlock < InitBlock
    %PATCHMATCHINGINITBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Access = protected)
        
        function [keypoints,landmarks,T_2W,secondIndex, candidates, ...
                prevFrameKeypoints] = run_(obj, fromIndex, T_1W)
        
            verboseDisp(obj.verbose, 'Bootstrapping...');
            
            image1 = obj.inputBlock.getImage(fromIndex);
            keypoints1 = obj.detector.extractFeatures(image1, obj.nKeypoints);
            descriptors1 = obj.detector.describeKeypoints(image1,keypoints1);
            
            errorTh = obj.errorThreshold^2;
            otherParams = [obj.K(:); T_1W(:); errorTh; obj.maxDistance];
            
            % Try bootstrapping with a couple of consecutive frames
            maxInliersCount = -1;
            for it = fromIndex+obj.nSkip:obj.nSkip:fromIndex+obj.nSkip*obj.nIt
                verboseDisp(obj.verbose, ...
                    'Evaluating frame %d for bootstrapping\n', it);
                
                image2 = obj.inputBlock.getImage(it);
                keypoints2 = obj.detector.extractFeatures(image2, obj.nKeypoints);
                descriptors2 = obj.detector.describeKeypoints(image2,keypoints2);
                
                [matches, p1_, p2_] = obj.detector.getMatches(...
                    descriptors1, descriptors2, keypoints1, keypoints2);
                unmatchedKeypoints_ = keypoints2(:, matches == 0);
                
                if size(p1_, 2) < 8 % Minimum number of parameters
                    verboseDisp(obj.verbose, ...
                        'Skipping frame %d due to too few matches (%d)', ...
                        [it, size(p1_, 2)]);
                    continue;
                end
                
                p1 = [p1_; ones(1,size(p1_,2))];
                p2 = [p2_; ones(1,size(p2_,2))];
                
                [model, inliers] = RANSAC(...
                    @relativePoseFromSample, @reprojectionErrorFromModel, ...
                    8, [p1; p2], obj.RANSACIt, otherParams, true, obj.adaptive);
                inliercount = nnz(inliers);

                if inliercount > maxInliersCount
                    maxInliersCount = inliercount;
                    secondIndex = it;
                    T_21 = model;
                    T_2W = T_21(1:3,:) * T_1W;
                    keypoints = p2(1:2, inliers);
                    prevFrameKeypoints = p1(1:2, inliers);
                    
                    % triangulation of valid landmarks (already filtered)
                    landmarks = triangulateFromPose(...
                        p1(:, inliers), p2(:, inliers), ...
                        T_21, obj.K, obj.K, T_1W);
                    
                    image2_ = image2;
                    
                    if inliercount >= obj.stopWithNPoints
                        break;
                    end
                end
            end
            
            candidates = [];
            if ~isempty(keypoints)
                mask = obj.detector.getMask(size(image2_), keypoints, 10);
                candidates = obj.detector.extractFeatures(image2_, obj.nCandidates, mask);
            end
        end
    end
end