classdef KLTInitBlock < InitBlock
    %KLTINITIALIZATIONBLOCK
    
    properties
        r_T = 20
        nItKLT = 50
        lambda = 0.1
        keyframeConfidence = 0.8
    end
    
    methods
        function obj = KLTInitBlock()
            obj.configurableProps = [obj.configurableProps, ...
                'r_T', 'nItKLT', 'lambda', 'keyframeConfidence'];
        end
    end
    
    methods (Access = protected)
        
        function [kp,P_W,T_2W,frameIdx,prevKp] = run_(obj, fromIdx, T_1W)
        
        verboseDisp(obj.verbose, 'Bootstrapping with KLT...');
        image1 = obj.inputBlock.getImage(fromIdx);
        
        kp1 = obj.detector.extractFeatures(image1, obj.nKeypoints);
            
        errorTh = obj.errorThreshold^2;
        otherParams = [obj.K(:); T_1W(:); errorTh; obj.maxDistance];
        kpold = [];
        kp = [];
        P_W = [];
        T_2W = [];
        frameIdx = fromIdx + obj.nSkip;
        prevKp = [];
        keep = [];
        keypoints = [];
        kpold = [];
        
        for it = fromIdx + obj.nSkip : fromIdx + obj.nSkip * obj.nIt
            verboseDisp(obj.verbose, ...
                'Evaluating frame %d for bootstrapping... ', it);
            
            image2 = obj.inputBlock.getImage(it);
                
            [keypoints_,keep_] = KLT(image1,image2,kp1,obj.r_T,obj.nItKLT,obj.lambda);
            kpold_ = kp1(:,keep_);
            verboseDisp(obj.verbose, ...
                'Confidence: %.3f\n', nnz(keep_) / numel(keep_));
            
            if ~isempty(keep) && nnz(keep_) / numel(keep_) < obj.keyframeConfidence
                break;
            end
            
            kpold = kpold_;
            keypoints = keypoints_;
            keep = keep_;
            frameIdx = it;
        end
            
        if size(kpold, 2) < 8 % Minimum number of parameters
            verboseDisp(obj.verbose, 'Unable to localize.', [it, size(kpold, 2)]);
            return;
        end

        p1 = [kpold; ones(1,size(kpold,2))];
        p2 = [keypoints; ones(1,size(keypoints,2))];

        [model, inliers] = RANSAC(...
            @relativePoseFromSample, @reprojectionErrorFromModel, ...
            8, [p1; p2], obj.RANSACIt, otherParams, true, obj.adaptive);
        
        T_21 = model;
        T_2W = T_21(1:3,:) * T_1W;

        kp = p2(1:2, inliers); %correct to use inliers
        prevKp = p1(1:2, inliers);

        % triangulation of valid landmarks (already filtered)
        P_W = triangulateFromPose(...
            p1(:, inliers), p2(:, inliers), T_21, obj.K, obj.K, T_1W);
        
        end
    end
end