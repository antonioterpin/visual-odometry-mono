classdef KLTInitBlock < InitBlock
    %KLTINITIALIZATIONBLOCK

    properties
        verbose = false
        RANSACIt = 2000
        nSkip = 1
        nIt = 3
        adaptive = 0.95
        stopWithNPoints = 90
        errorThreshold = 1
        maxDistance = 200
        
        klt = Klt;
    end
    
    properties (Constant)
        configurableProps = {'verbose', 'RANSACIt', 'adaptive', ...
            'nSkip', 'nIt', 'errorThreshold', 'maxDistance', ...
            'stopWithNPoints'}
    end
    
    methods (Access = protected)
        
        function [keypoints,landmarks, descriptors,T_2W,secondIndex, ...
                unmatchedKeypoints, unmatchedDescriptors, ...
                prevFrameKeypoints] = ...
            run_(obj, input, K, fromIndex, T_1W)
        
        descriptors = [];
        unmatchedDescriptors = [];
        
        verboseDisp(obj.verbose, 'Bootstrapping...');
        image1 = input.getImage(fromIndex);
        
        [keypoints1, ~] = obj.Detector.extractFeatures(image1);
            
        errorTh = obj.errorThreshold^2;
        otherParams = [K(:); T_1W(:); errorTh; obj.maxDistance];
        
        maxInliersCount = -1;
        for it = fromIndex+obj.nSkip : obj.nSkip : fromIndex+obj.nSkip*obj.nIt
                verboseDisp(obj.verbose, ...
                    'Evaluating frame %d for bootstrapping\n', it);
                                
            [kpold, keypoints, keypointsLost, keep] = obj.klt.KltTracker(input,...
                it, keypoints1);
            
            if size(kpold, 2) < 8 % Minimum number of parameters
                    verboseDisp(obj.verbose, ...
                        'Skipping frame %d due to too few matches (%d)', ...
                        [it, size(kpold, 2)]);
                    continue;
            end
            
            p1 = [kpold; ones(1,size(kpold,2))];
            p2 = [keypoints; ones(1,size(keypoints,2))];
            
            [model, inliers] = RANSAC(...
                @relativePoseFromSample, @reprojectionErrorFromModel, ...
                8, [p1; p2], obj.RANSACIt, otherParams, true, obj.adaptive);
            
            keep(~inliers) = 0;     %check this
            inliercount = nnz(keep);
            
            if inliercount > maxInliersCount
                    maxInliersCount = inliercount;
                    secondIndex = it;
                    T_21 = model;
                    T_2W = T_21(1:3,:) * T_1W;
                    
                    unmatchedKeypoints = [keypointsLost, keypoints(:, ~inliers)];
                    
                    keypoints = p2(1:2, inliers); %correct to use inliers
                    prevFrameKeypoints = p1(1:2, inliers);
                    
                    % triangulation of valid landmarks (already filtered)
                    landmarks = triangulateFromPose(...
                        p1(:, inliers), p2(:, inliers), T_21, K, K, T_1W);
                    
                    if inliercount >= obj.stopWithNPoints
                        break;
                    end
            end
        end
        
        end
    end
end