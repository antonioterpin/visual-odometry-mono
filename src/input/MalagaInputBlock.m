classdef MalagaInputBlock < InputBlock
    %MALAGAINPUTBLOCK Handles Malaga dataset
    
    properties (Access = private)
        LeftImages 
        ImageNameFormat string
        Path string
    end
    
    methods
        function obj = MalagaInputBlock(path)
            obj.Path = path;
            obj.IsKConstant = true;
        end
        
        function obj = init(obj)
            obj.K = obj.loadIntrinsics();
            obj.ImageNameFormat = [path ...
            'malaga-urban-dataset-extract-07_rectified_800x600_Images/%s'];
            obj.LeftImages = dir([path ...
            'malaga-urban-dataset-extract-07_rectified_800x600_Images/']);
            obj.LeftImages = obj.LeftImages(3:2:end);
            obj.NumberOfImages = length(obj.LeftImages);
        end
    end
    
    methods (Access = protected)
        function image = getImage_(obj, imageIndex)
            currentImage = obj.LeftImages(imageIndex);
            image = rgb2gray(imread(sprintf(obj.ImageNameFormat, ...
                currentImage.name)));
        end
    end
    
    methods (Access = private)
        function K = loadIntrinsics(obj)
            %loadIntrinsics Loads the instrinsics parameters
            % hardcoded 
            K = [621.18428 0 404.0076
                    0 621.18428 309.05989
                    0 0 1];
        end
    end
end

