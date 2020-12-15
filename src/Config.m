classdef Config
    %CONFIG Set the configurations you want to try
    %   Set the experiment you want to test on the json file
    
    properties
        InputHandler % InputBlock = KittiInputBlock('');
        DetectorHandler % DetectorBlock = HarrisDetectorBlock({})
        InitializationHandler % InitBlock = PatchMatchingInitBlock();
        ContinuousOperationHandler = []
        OptimizationHandler = []
        OutputHandler = []
    end
    
    methods (Static)
        function obj = loadFromFile(filename)
            %CONFIG Construct an instance of this class 
            obj = Config();
            
            configuration = Config.decodeJsonFile(filename);
            assert(isfield(configuration, 'InputBlock'), ...
                'Missing dataset information in the configuration file.');
            assert(isfield(configuration, 'DetectorBlock'), ...
                ['Missing detector block information ' ...
                    'in the configuration file.']);
            assert(isfield(configuration, 'InitBlock'), ...
                ['Missing initialization block information ' ...
                    'in the configuration file.']);
            assert(isfield(configuration, 'COBlock'), ...
                ['Missing continuous operation block information ' ...
                    'in the configuration file.']);
            assert(isfield(configuration, 'OptBlock'), ...
                ['Missing optimization block information ' ...
                    'in the configuration file.']);
            assert(isfield(configuration, 'OutputBlock'), ...
                ['Missing output block information ' ...
                    'in the configuration file.']);
                
            obj.InputHandler ...
                = Config.extractInputBlock(configuration.InputBlock);
            obj.DetectorHandler ...
                = Config.extractDetectorBlock(configuration.DetectorBlock);
            obj.InitializationHandler = Config.extractInitBlock(...
                configuration.InitBlock, obj.DetectorHandler);
            obj.ContinuousOperationHandler ...
                = Config.extractCOBlock(configuration.COBlock);
            obj.OptimizationHandler ...
                = Config.extractOptBlock(configuration.OptBlock);
            obj.OutputHandler ...
                = Config.extractOutputBlock(configuration.OutputBlock);
        end 
        
        function inputBlock = extractInputBlock(datasetInfo)
            assert(isfield(datasetInfo, 'Handler'), ...
                'The name of the handler is required.');
            assert(isfield(datasetInfo, 'Path'), ...
                'Path to the root folder of the dataset is required.');
            
            try
                datasetName = datasetInfo.Handler;
                datasetHandlerClassname ....
                    = sprintf('%sInputBlock', datasetName);
                fprintf('Handling dataset through class %s.\n', ...
                    datasetHandlerClassname);

                % Create instance of the dataset handler
                inputBlock = feval(...
                    datasetHandlerClassname, datasetInfo.Path);
                inputBlock = inputBlock.init();
            catch exception
                error(['Error loading dataset. ', ...
                    'Make sure the provided handler is S, ', ...
                    'where {S}InputBlock is a valid handler.']);
            end
        end
        
        function initBlock = extractInitBlock(initBlockInfo, detectorHandler)
            assert(isfield(initBlockInfo, 'Handler'), ...
                'The name of the handler is required.');
            
            try
                initBlockName = initBlockInfo.Handler;
                initHandlerName ....
                    = sprintf('%sInitBlock', initBlockName);
                fprintf('Handling dataset through class %s.\n', ...
                    initHandlerName);

                initBlock = feval(initHandlerName);
                if isfield(initBlockInfo, 'Params')
                    initBlock.setParams(initBlockInfo.Params);
                end
            catch exception
                exception
                error(['Error loading init block. ', ...
                    'Make sure the provided handler is S, ', ...
                    'where {S}InitBlock is a valid handler.']);
            end
            
            initBlock.Detector = detectorHandler;
        end
        
        function detectorBlock = extractDetectorBlock(detectorBlockInfo)
            assert(isfield(detectorBlockInfo, 'Handler'), ...
                'The name of the handler is required.');
            
            try
                detectorBlockName = detectorBlockInfo.Handler;
                detectorHandlerName ....
                    = sprintf('%sDetectorBlock', detectorBlockName);
                fprintf('Detecting features through class %s.\n', ...
                    detectorHandlerName);

                params = {};
                if isfield(detectorBlockInfo, 'Params')
                    params = detectorBlockInfo.Params;
                end
                detectorBlock = feval(detectorHandlerName, params);
            catch exception
                error(['Error loading detector block. ', ...
                    'Make sure the provided handler is S, ', ...
                    'where {S}DetectorBlock is a valid handler.']);
            end
        end
        
        function coBlock = extractCOBlock(coBlockInfo)
            coBlock = [];
        end
        
        function optBlock = extractOptBlock(optBlockInfo)
            optBlock = [];
        end
        
        function outBlock = extractOutputBlock(optBlockInfo)
            outBlock = [];
        end
    end
    
    methods (Static = true)
        function obj = decodeJsonFile(filename)
            %DECODEJSONFILE Open the JSON file and transfor it in the json
            % structure. Return a json object
            fid = fopen(filename);
            json = char(fread(fid).');
            obj = jsondecode(json);
        end
    end
end

