classdef (Abstract) OptimizationBlock < handle
    %OPTIMIZATIONBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        K
        
        verbose = true
        MaxIter = 20
        isActive = true
        everyNIterations = 100
        maxBundleSize = 150
        plotMap = 5
        plotSparsityPattern = 6
        configurableProps = {'verbose', 'MaxIter', 'isActive', ...
            'everyNIterations', 'maxBundleSize', 'plotMap', 'plotSparsityPattern'}
    end
    
    methods
        function hiddenState = optimize(obj, hiddenState, observations)
            if obj.plotMap > 0
                figure(obj.plotMap);
                subplot(1,2,1);
                OptimizationBlock.plotProblem(hiddenState, observations);
                title('Problem before bundle adjustment');
            end
            
            e = @(hiddenState) obj.error(hiddenState, observations, obj.K);
            J = obj.getJacobPattern(hiddenState, observations);
            
            verboseDisp(obj.verbose, 'Starting optimization on bundle...');
            options = optimoptions(@lsqnonlin, ...
                'Display', 'iter', ...
                'MaxIter', obj.MaxIter, ...
                'JacobPattern', J);
            hiddenState = lsqnonlin(e, hiddenState, [], [], options);
            
            if obj.plotMap > 0
                figure(obj.plotMap);
                subplot(1,2,2);
                OptimizationBlock.plotProblem(hiddenState, observations);
                title('Problem after bundle adjustment');
            end
            
            if obj.plotSparsityPattern > 0
                figure(obj.plotSparsityPattern);
                spy(J); 
            end
        end
    end
    
    methods (Static = true)    
        function plotProblem(hiddenState, observations, range)
            n = observations(1);
            T_W_frames = reshape(hiddenState(1:n*6), 6, []);
            p_W_landmarks = reshape(hiddenState(n*6+1:end), 3, []);

            p_W_frames = zeros(3, n);
            for i = 1:n
                T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
                p_W_frames(:, i) = T_W_frame(1:3, end);
            end

            plot(p_W_landmarks(1, :), p_W_landmarks(3, :), '.');
            hold on;
            plot(p_W_frames(1, :), p_W_frames(3, :), 'rx', 'Linewidth', 3);
            hold off;

            if nargin > 2
                axis equal;
                axis(range);
            end
        end
    end
    
    methods (Abstract)
        e = error(obj, hiddenState, observations, K)
        J = getJacobPattern(obj, hiddenState, observations)
        options = setOptions(obj, options)
    end
end