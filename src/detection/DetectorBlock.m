classdef (Abstract) DetectorBlock < handle
    %DETECTORBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        plotKeypoints = 0;
        plotMask = 0;
        configurableProps = { 'plotMask', 'plotKeypoints' }
    end
    
    methods
        function [matchesIndices, p1, p2] = ...
            getMatches(obj, descriptors1, descriptors2, keypoints1, keypoints2)
        % GETMATCHES Finds matches between of the given described keypoints.
        %
        % matchesIndices = obj.GETMATCHES(descriptors1,descriptors2) matches
        % descriptors1 and descriptors2 to find matching indices.
        % descriptors1 is MxN1, the descriptors of N1 keypoints in the first image.
        % descriptors2 is MxN2, the descriptors of N2 keypoints in the second
        % image.
        % matchesIndices is N1x1, matchesIndices(i) == 0 iff there is no matching
        % in image1 for the i-th keypoint in image2. Otherwise, matchesIndices(i)
        % is equal to the corresponding keypoint in image1.
        %
        % [..., p1, p2] = obj.GETMATCHES(..., keypoints1, keypoints2) provided the
        % keypoints corresponding to the given descriptors, returns also the
        % correspondences.
        % keypoints1 is 2xN1, the image1 coordinates associated to descriptors1[u; v];
        % keypoints2 is 2xN2, the image2 coordinates associated to descriptors2[u; v];
        % p1 is 3xN, the homogenous image1 coordinates with a correspondence in
        % image2 [u; v; 1], N is the number of correspondences.
        % p2 is 3xN, the homogenous image2 coordinates with a correspondence in
        % image1 [u; v; 1], N is the number of correspondences.

        arguments
            obj DetectorBlock
            descriptors1
            descriptors2
            keypoints1 {mustBeSameAmount(keypoints1,descriptors1)} = []
            keypoints2 {mustBeSameAmount(keypoints2,descriptors2)} = []
        end
        
        [matchesIndices, p1, p2] = obj.getMatches_(...
            descriptors1,descriptors2,keypoints1,keypoints2);
        
        end
        
        function keypoints = extractFeatures(obj, image, nKeypoints, mask)
        % EXTRACTFEATURES Extracts features from an image
        %
        % TODO update with mask explanation
        %
        % [keypoints, descriptors] = obj.EXTRACTFEATURES(image) returns the found
        % keypoints and the corresponding descriptors for the given image.
        % keypoints is 2xN, image coordinates of the found keypoints [u; v].
        % descriptors is MxN, descriptors of the found keypoints.
        
        % extract keypoints and descriptors from different parts of the
        % image, according to the distribution provided
        
            if nargin < 4
                mask = ones(size(image));
            end

            width = size(image,2);
            height = size(image,1);

            nHBlocks = size(nKeypoints, 2);
            hBlocksSize = ceil(width / nHBlocks);

            nVBlocks = size(nKeypoints, 1);
            vBlocksSize = ceil(height / nVBlocks);

            % tl (top left) indeces
            uIdx = (0:(nHBlocks-1)) * hBlocksSize + 1;
            vIdx = (0:(nVBlocks-1)) * vBlocksSize + 1;

            [tl_u, tl_v] = meshgrid(uIdx, vIdx);
            tl_u = tl_u(:); 
            tl_v = tl_v(:);

            % br (bottom right) indeces
            br_u = min(tl_u + hBlocksSize, width);
            br_v = min(tl_v + vBlocksSize, height);

            % TODO could be done in parfor
            keypoints = [];

            if obj.plotKeypoints > 0
                figure(obj.plotKeypoints);
                imshow(image);
                hold on;
            end
            for blockIdx = 1:numel(tl_u)
                tl = [tl_u(blockIdx); tl_v(blockIdx)];
                br = [br_u(blockIdx); br_v(blockIdx)];
                crop = image(tl(2):br(2),tl(1):br(1));
                crop_mask = mask(tl(2):br(2),tl(1):br(1));
                kp = obj.extractFeatures_(crop,nKeypoints(blockIdx),crop_mask);
                kp = kp + tl - 1;
                if obj.plotKeypoints > 0
                    plot(kp(1,:), kp(2,:), 'x');
                end
                keypoints = [keypoints, kp];
            end
            if obj.plotKeypoints > 0
                hold off;
            end
        end
        
        function descriptors = describeKeypoints(obj,image, keypoints)
        % TODO DOCUMENT
        
        arguments
            obj DetectorBlock
            image
            keypoints (2,:)
        end
        
        descriptors = obj.describeKeypoints_(image,keypoints);
        end
        
        function mask = getMask(obj, imageSize, kp, suppressionRadius)
            mask = ones(imageSize);
            mask = padarray(mask, [suppressionRadius, suppressionRadius]);
            
            suppressPatch = zeros(2*suppressionRadius+1);
            range = -suppressionRadius:suppressionRadius;
            kp = kp + suppressionRadius;
            for i = 1:size(kp,2)
                mask(range + kp(2,i), range + kp(1,i)) = suppressPatch;
            end
            
            mask = mask(suppressionRadius+1:end-suppressionRadius,...
                suppressionRadius+1:end-suppressionRadius);
            
            if obj.plotMask
                spy(mask);
            end
        end
    end
    
    methods (Access = protected, Abstract)
        [matchesIndices, p1, p2] = getMatches_(obj,...
            descriptors1, descriptors2, keypoints1, keypoints2)
        keypoints = extractFeatures_(obj,image,nFeatures,mask)
        descriptors = describeKeypoints_(obj,image,keypoints);
    end
end

function mustBeSameAmount(a,b)
    if size(a,1) ~= 0 && size(a,2) ~= size(b,2)
        eid = 'Size:notEqual';
        msg = 'Each keypoint must have a corresponding descriptor.';
        throwAsCaller(MException(eid,msg))
    end
end

