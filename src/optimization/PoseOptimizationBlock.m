classdef PoseOptimizationBlock < OptimizationBlock
    %BAOPTIMIZATIONBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    methods
        function e = error(~, hiddenState, observations, K)
            n = observations(1);
    
            T = reshape(hiddenState(1:6*n), 6, []); % twist vectors
            P_W = reshape(hiddenState(6 * n + 1:end), 3, []); % world points
            P_W = [P_W; ones(1, size(P_W, 2))];

            e = zeros(2, (numel(observations) - 2 - n) / 3);
            
            poses = zeros(3,n);
            for i = 1 : n
                T_Wi = twist2HomogMatrix(T(:,i));
                poses(:,i) = T_Wi(1:3,end);
            end

            poses(1,:) = smooth(poses(1,:), 10);
            poses(2,:) = smooth(poses(2,:), 10);
            poses(3,:) = smooth(poses(3,:), 10);

            j = 3;
            k = 1;
            for i = 1 : n
                k_i = observations(j);
                if k_i > 0
                    p_i = reshape(observations(j+1:j+2*k_i), 2, []);
                    l_i = observations(j+2*k_i+1:j+3*k_i);
                    j = j + 3*k_i + 1;

                    T_Wi = twist2HomogMatrix(T(:,i));
                    P_i = T_Wi \ P_W(:, l_i);
                    repr_i = projectPoints(P_i(1:3,:), K);

                    e(:,k:k+k_i-1) = (p_i - repr_i) * (1 +  ...
                        + obj.smooth * vecnorm(poses(:,i) - T_Wi(1:3,end),2));
                end
                
                k = k + k_i;
            end
        end
        
        function options = setOptions(obj, options)
%             options.Algorithm = obj.Algorithm;
%             options.OptimalityTolerance = obj.OptimalityTolerance;
%             options.UseParallel = obj.UseParallel;
        end
        
        function J = getJacobPattern(~, hiddenState, observations)
            n = observations(1);
            N = (numel(observations) - 2 - n)/3;
            n_error_terms = 2 * N;

            J = spalloc(n_error_terms, numel(hiddenState), (6+3) * n_error_terms);

            j = 3;
            e_i = 1;
            for i = 1:n
                k_i = observations(j);
                l_i = observations(j+2*k_i+1:j+3*k_i);
                j = j + 3*k_i + 1;

                J(e_i : e_i+2*k_i-1, 6*(i-1)+1:6*i) = 1;

%                 for m = 1:numel(l_i)
%                     J(e_i+(m-1)*2:e_i+m*2-1, 1+n*6+(l_i(m)-1)*3:n*6+l_i(m)*3) = 1;
%                 end

                e_i = e_i + 2*k_i;
            end
        end
    end
    
    methods (Static = true)
    end
end