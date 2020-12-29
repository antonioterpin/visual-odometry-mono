classdef (Abstract) DetectorBlock
    %DETECTORBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        distribution = 1;
        configurableProps = { 'distribution' }
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
        
        function [keypoints, descriptors] = extractFeatures(obj, image)
        % EXTRACTFEATURES Extracts features from an image
        %
        % [keypoints, descriptors] = obj.EXTRACTFEATURES(image) returns the found
        % keypoints and the corresponding descriptors for the given image.
        % keypoints is 2xN, image coordinates of the found keypoints [u; v].
        % descriptors is MxN, descriptors of the found keypoints.

        [keypoints, descriptors] = obj.extractFeatures_(image);
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
    end
    
    methods (Access = protected, Abstract)
        [matchesIndices, p1, p2] = getMatches_(obj,...
            descriptors1, descriptors2, keypoints1, keypoints2)
        [keypoints, descriptors] = extractFeatures_(obj,image)
        [descriptors] = describeKeypoints_(obj,image,keypoints);
    end
end

function mustBeSameAmount(a,b)
    if size(a,1) ~= 0 && size(a,2) ~= size(b,2)
        eid = 'Size:notEqual';
        msg = 'Each keypoint must have a corresponding descriptor.';
        throwAsCaller(MException(eid,msg))
    end
end

