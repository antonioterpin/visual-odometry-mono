classdef KLTInitBlock < InitBlock
    %KLTINITIALIZATIONBLOCK
    
    properties
        r_T = 20
        nItKLT = 50
        lambda = 0.1
        nLevels = 3
    end
    
    methods
        function obj = KLTInitBlock()
            obj.configurableProps = [obj.configurableProps, ...
                'r_T', 'nItKLT', 'lambda', 'nLevels'];
        end
    end
    
    methods (Access = protected)
        
        function [kp,P_W,T_2W,prevKp,tracker] = run_(obj, idx1, idx2, T_1W)
        
        verboseDisp(obj.verbose, 'Bootstrapping with KLT...');
        image1 = obj.inputBlock.getImage(idx1);
        kp1 = obj.detector.extractFeatures(image1, obj.nKeypoints);

        tracker = vision.PointTracker(...
            'MaxBidirectionalError',obj.lambda, ...
            'BlockSize', [obj.r_T, obj.r_T], ...
            'MaxIterations', obj.nItKLT, ...
            'NumPyramidLevels', obj.nLevels);
        initialize(tracker,kp1.',image1);
        
        for it = idx1+1 : idx2
            image2 = obj.inputBlock.getImage(it);
            [points,validity] = tracker(image2);
        end
        
        N = nnz(validity);
        
        if N < 8 % Minimum number of parameters
            verboseDisp(obj.verbose, 'Unable to localize.\n',[]);
            return;
        end
        
        verboseDisp(obj.verbose, 'Next keyframe is %d.\n', idx2);

        % RANSAC + 8Point
        
        p1 = [kp1(:,validity); ones(1,N)];
        p2 = [points(validity,:).'; ones(1,N)];
        
        errorTh = obj.errorThreshold^2;
        otherParams = [obj.K(:); T_1W(:); errorTh; obj.maxDistance];

        [model, inliers] = RANSAC(...
            @relativePoseFromSample, @reprojectionErrorFromModel, ...
            8, [p1; p2], obj.RANSACIt, otherParams, obj.verbose, obj.adaptive);
        
        T_21 = model;
        T_2W = T_21(1:3,:) * T_1W;

        kp = p2(1:2, inliers);
        prevKp = p1(1:2, inliers);

        % triangulation of valid landmarks (already filtered)
        P_W = triangulateFromPose(...
            p1(:, inliers), p2(:, inliers), T_21, obj.K, obj.K, T_1W);
        
        end
    end
end