classdef ParkingInputBlock < InputBlock
    %PARKINGINPUTBLOCK Handles Parking dataset
    
    properties (Access = private)
        ImageNameFormat string
        Path string
    end
    
    methods
        function obj = ParkingInputBlock(path)
            obj.Path = path;
            obj.IsKConstant = true;
        end
        
        function obj = init(obj)
            obj.K = obj.loadIntrinsics();
            obj.ImageNameFormat = ...
                sprintf('%s/images/img_%05d.png', obj.Path);
            obj.NumberOfImages = 598;
        end
    end
    
    methods (Access = protected)
        function image = getImage_(obj, imageIndex)
            image = rgb2gray(imread(...
                sprintf(obj.ImageNameFormat, imageIndex)));
        end
        
        function K = loadIntrinsics(obj)
            %loadIntrinsics Loads the instrinsics parameters
            % we select the first row
            K = load(sprintf('%s/K.txt', obj.Path));
        end
    end
end