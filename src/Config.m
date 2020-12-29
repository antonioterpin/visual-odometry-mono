classdef Config
    %CONFIG Set the configurations you want to try
    %   Set the experiment you want to test on the json file
    
    properties
        InputHandler
        DetectorHandler
        InitializationHandler
        ContinuousOperationHandler = []
        OptimizationHandler = []
        OutputHandler = []
        PipelineParams = {};
    end
    
    methods (Static)
        function obj = loadFromFile(filename)
            %CONFIG Construct an instance of this class 
            obj = Config();
            
            configuration = Config.decodeJsonFile(filename);
            mandatoryFields = { 'InputBlock', 'DetectorBlock', ...
                'InitBlock', 'COBlock', 'OptBlock', 'OutputBlock' };
            missingFields = mandatoryFields(~isfield(configuration, mandatoryFields));
            assert(isempty(missingFields), ...
                sprintf('Missing %s information in the configuration file\n', ...
                    strjoin(missingFields, ', ')));
                
            obj.InputHandler ...
                = Config.extractInputBlock(configuration.InputBlock);
            K = obj.InputHandler.getIntrinsics();
            
            obj.DetectorHandler ...
                = Config.extractDetectorBlock(configuration.DetectorBlock);
            
            obj.InitializationHandler = Config.extractInitBlock(...
                configuration.InitBlock, obj.DetectorHandler);
            
            obj.ContinuousOperationHandler ...
                = Config.extractCOBlock(configuration.COBlock, obj.DetectorHandler, K);
            
            obj.OptimizationHandler ...
                = Config.extractOptBlock(configuration.OptBlock, K);
            
            obj.OutputHandler ...
                = Config.extractOutputBlock(configuration.OutputBlock, obj.InputHandler);
            
            % Pipeline specific config parameters
            if isfield(configuration, 'Pipeline')
                obj.PipelineParams = configuration.Pipeline;
            end
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
                inputBlock.init();
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
                fprintf('Handling initialization through class %s.\n', ...
                    initHandlerName);

                initBlock = feval(initHandlerName);
            catch exception
                error(['Error loading init block. ', ...
                    'Make sure the provided handler is S, ', ...
                    'where {S}InitBlock is a valid handler.']);
            end
            
            if isfield(initBlockInfo, 'Params') && isprop(initBlock, 'configurableProps')
                Config.setParams(initBlock, initBlockInfo.Params);
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

                detectorBlock = feval(detectorHandlerName);
                if isfield(detectorBlockInfo, 'Params')
                    Config.setParams(detectorBlock, detectorBlockInfo.Params);
                end
            catch exception
                error(['Error loading detector block. ', ...
                    'Make sure the provided handler is S, ', ...
                    'where {S}DetectorBlock is a valid handler.']);
            end
        end
        
        function coBlock = extractCOBlock(coBlockInfo, detectorHandler, K)
            assert(isfield(coBlockInfo, 'Handler'), ...
                'The name of the handler is required.');
            
            try
                coBlockName = coBlockInfo.Handler;
                coHandlerName ....
                    = sprintf('%sCOBlock', coBlockName);
                fprintf('Handling continuous operation through class %s.\n', ...
                    coHandlerName);

                coBlock = feval(coHandlerName);
            catch exception
                error(['Error loading continuous operation block. ', ...
                    'Make sure the provided handler is S, ', ...
                    'where {S}COBlock is a valid handler.']);
            end
            
            if isfield(coBlockInfo, 'Params') && isprop(coBlock, 'configurableProps')
                Config.setParams(coBlock, coBlockInfo.Params);
            end
            
            coBlock.Detector = detectorHandler;
            coBlock.K = K;
        end
        
        function optBlock = extractOptBlock(optBlockInfo, K)
            assert(isfield(optBlockInfo, 'Handler'), ...
                'The name of the handler is required.');
            
            try
                handlerName = optBlockInfo.Handler;
                handlerClassname ....
                    = sprintf('%sOptimizationBlock', handlerName);
                fprintf('Handling optimization through class %s.\n', ...
                    handlerClassname);

                % Create instance of the dataset handler
                optBlock = feval(handlerClassname);
            catch exception
                error(['Error loading optimization block. ', ...
                    'Make sure the provided handler is S, ', ...
                    'where {S}OptimizationBlock is a valid handler.']);
            end
            
            if isfield(optBlockInfo, 'Params')
                Config.setParams(optBlock, optBlockInfo.Params);
            end
            
            optBlock.K = K;
        end
        
        function outBlock = extractOutputBlock(outBlockInfo, inputHandler)
            outBlock = OutputBlock;
            Config.setParams(outBlock, outBlockInfo);
            outBlock.inputHandler = inputHandler;
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
        
        function obj = setParams(obj, params)
            assert(isprop(obj, 'configurableProps'), ...
                'obj must have a configurableProps field');
            
            configuredProps = obj.configurableProps(...
                isfield(params, obj.configurableProps));
            for prop = configuredProps
                obj.(prop{:}) = params.(prop{:});
            end
        end
    end
end

