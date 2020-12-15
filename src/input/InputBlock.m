classdef (Abstract) InputBlock
    % INPUTBLOCK Abstract class to handle a generic dataset.
    %
    % To add a dataset, inherit from this class. 
    % InputBlock.K and InputBlock.NumberOfImages have to be set in the
    % constructor. Overload the protected method getImage_ to define how
    % images are loaded from the new dataset.
    
    properties %(Access = protected)
        K % Camera intrinsics for the specific dataset
        NumberOfImages = 0 % Number of images in the considered dataset
        IsKConstant = true % Are the intrinsics constant?
    end
    
    methods
        function intrinsics = getIntrinsics(obj)
            % GETINTRINSICS Get camera intrinsics for the specific dataset.
            %
            % intrinsics = obj.GETINTRINSICS() returns the K matrix of the
            % camera for the considered dataset.
            
            if ~obj.IsKConstant
                obj.K = loadIntrinsics(obj);
            end
            intrinsics = obj.K;
        end
        
        function image = getImage(obj, imageIndex)
            % GETIMAGE Get image at the given index
            %
            % image = obj.GETIMAGE(imageIndex) returns the image at index
            % imageIndex in the dataset. imageIndex has to be less or
            % equal to obj.NumberOfImages.
            
            if imageIndex > obj.NumberOfImages
                InputBlock.imageIndexOutOfBounds(...
                    imageIndex, obj.NumberOfImages);
            end
            image = getImage_(obj, imageIndex);
        end
        
        function NumberOfImages = getNumberOfImages(obj)
            % GETNUMBEROFIMAGES Get number of images in the dataset.
            %
            % NumberOfImages = obj.GETNUMBEROFIMAGES() returns the number
            % of images in the considered dataset.
            
            NumberOfImages = obj.NumberOfImages;
        end
    end
    
    methods (Abstract)
        obj = init(obj)
    end
    
    methods (Access = protected, Abstract)
        % Method to overload to get next image from dataset
        image = getImage_(obj, imageIndex);
        % Method to overload to load intrinsics from dataset
        K = loadIntrinsics(obj);
    end
    
    methods (Access = protected, Static = true)
        % Common messages
        function calibrationFileNotFound()
            error('Calibration file not found');
        end
        
        function imageIndexOutOfBounds(imageIndex, numberOfImages)
            error('Image index %d out of bounds. ' + ...
                'There are %d images in this dataset.', ...
                imageIndex, numberOfImages);
        end
    end
end

