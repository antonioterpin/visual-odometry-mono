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
        
        function [kp,P_W,T_2W,frameIdx,prevKp,tracker] = run_(obj, fromIdx, T_1W)
        
        verboseDisp(obj.verbose, 'Bootstrapping with KLT...');
        image1 = obj.inputBlock.getImage(fromIdx);
        kp1 = obj.detector.extractFeatures(image1, obj.nKeypoints);

        tracker = vision.PointTracker(...
            'MaxBidirectionalError',obj.lambda, ...
            'BlockSize', [obj.r_T, obj.r_T], ...
            'MaxIterations', obj.nItKLT, ...
            'NumPyramidLevels', 3);
        initialize(tracker,kp1.',image1);
            
        errorTh = obj.errorThreshold^2;
        otherParams = [obj.K(:); T_1W(:); errorTh; obj.maxDistance];
        kp = [];
        P_W = [];
        T_2W = [];
        frameIdx = fromIdx + obj.nSkip;
        prevKp = [];
        keep = ones(size(kp1,2), obj.lambda);
        keypoints = [];
        
        for it = fromIdx + obj.nSkip : fromIdx + obj.nSkip * obj.nIt
            verboseDisp(obj.verbose, ...
                'Evaluating frame %d for bootstrapping... ', it);

            image2 = obj.inputBlock.getImage(it);
            [points,validity] = tracker(image2);
            
            verboseDisp(obj.verbose, 'Confidence: %.3f\n', nnz(validity) / numel(validity));
            
            if ~isempty(keypoints) && nnz(validity) / numel(validity) < obj.keyframeConfidence
                break;
            end
            
            keypoints = points(validity,:).';
            keep = validity;
            frameIdx = it;
        end
            
        if nnz(keep) < 8 % Minimum number of parameters
            verboseDisp(obj.verbose, 'Unable to localize.\n',[]);
            return;
        end
        
        verboseDisp(obj.verbose, 'Next keyframe is %d.\n', frameIdx);

        p1 = [kp1(:,keep); ones(1,size(keypoints,2))];
        p2 = [keypoints; ones(1,size(keypoints,2))];

        [model, inliers] = RANSAC(...
            @relativePoseFromSample, @reprojectionErrorFromModel, ...
            8, [p1; p2], obj.RANSACIt, otherParams, obj.verbose, obj.adaptive);
        
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