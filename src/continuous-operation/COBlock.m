classdef (Abstract) COBlock < handle
    %COBLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector
        inputBlock
        
        K
        p3pRANSACIt = 2000
        p3pTolerance = 3
        minInliers = 30
        adaptive = 0
        verbose = false
        
        configurableProps = { 'p3pRANSACIt', 'p3pTolerance', ...
            'verbose', 'minInliers', 'adaptive'}
    end
    
    methods
        function [R_CW, t_CW, trKp, kpMask, trKpc, kpcMask, newKpc] = ...
                localize(obj, prevFrameIdx, frameIdx, kp, P_W, kpc)
            
            % 1. Keyframe selection & tracking
%             nKp = size(kp, 2);
%             [trKpKpc, kpMask, newKpc] = obj.track(prevFrameIdx, frameIdx, [kp; kp_c]);
            [trKp, kpMask, newKpc] = obj.track(prevFrameIdx, frameIdx, kp);
            kpcMask = [];
            trKpc = [];
            
%             trKpMask = kpMask(1:nKp);
%             trKpcMask = kpMask(nKp+1:end);
            
%             trKp = trKpKpc(:, 1:nnz(kpMask));
%             trKpc = trKpKpc(:, nnz(kpMask)+1:end);
            
            verboseDisp(obj.verbose, ...
                'Found %d matches.\n', size(trKp, 2));
            
            % 2. Localization
            if size(trKp, 2) >= obj.minInliers
               [R_CW, t_CW, inliers] = p3pRANSAC(...
                    trKp, P_W(:,kpMask), obj.K, ...
                    obj.p3pRANSACIt, obj.p3pTolerance, ...
                    obj.minInliers, obj.adaptive, obj.verbose); 
                
                % 3. Output
                kpMask(kpMask > 0) = inliers;
                trKp = trKp(:,inliers);
            else
                kpMask = [];
                newKpc = [];
                trKp = [];
                R_CW = []; 
                t_CW = [];
                verboseDisp(obj.verbose, ...
                    'Too few matches to localize.\n', []);
            end
        end
    end
    
    methods (Abstract) %Access = protected
        [trKp, kpMask, newKpc] = track(obj, prevFrameIdx, frameIdx, kp);
    end
end

