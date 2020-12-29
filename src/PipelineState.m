classdef PipelineState < handle
    % PIPELINESTATE Embedds the pipeline state, with the history of poses
    % and 
    % 
    %   Detailed explanation goes here
    
    properties % (Access = private)
        Poses = table([],[],[],[], 'VariableNames', {'Id', 'R_CW', 't_CW', 'Position'})
        Landmarks = table([],[], 'VariableNames', {'Id', 'Position'})
        ObservationGraph = graph % Poses x Landmarks -> (keypoints)
        Candidates = table([],[],[],...
            'VariableNames', {'PoseId', 'Keypoints', 'Descriptors'})
%         K % The intrinsics matrix
        lastLandmark = 0;
        lost = true;
        descriptorSize = 0;
    end
    
    % Config params
    properties (Constant)
        configurableProps = {'lostBelow', 'verbose', ...
            'cosineThreshold', 'candidatesWindowSize'}
    end
    properties
        lostBelow = 20
        verbose = true
        cosineThreshold = 0.1 % Threshold for triangulation 
        candidatesWindowSize = 3
    end
    
    methods
        function lost = isLost(state, lost)
            if nargin > 1
                state.lost = lost;
            else
                poseNode = find(state.ObservationGraph.Nodes.PoseId == state.Poses.Id(end));
                state.lost = state.lost || degree(state.ObservationGraph, poseNode) < state.lostBelow;
            end
            
            lost = state.lost;
        end
        
        function [R_CW, t_CW] = getLastPose(state)
            R_CW = reshape(state.Poses.R_CW(end,:), 3, 3);
            t_CW = reshape(state.Poses.t_CW(end,:), 3, 1);
        end
        
        function prevPoseIdx = gotoPrevPose(state)
            prevPoseIdx = [];
            if size(state.Poses, 1) > 1
                prevPoseIdx = state.Poses.Id(end - 1);
                state.resetToPose(prevPoseIdx);
            end
        end
        
        function [R_CW, t_CW] = getPose(state, poseIdx)
            idx = find(state.Poses.Id == poseIdx);
            
            R_CW = reshape(state.Poses.R_CW(idx,:), 3, 3);
            t_CW = reshape(state.Poses.t_CW(idx,:), 3, 1);
        end
        
        function N = getNumberOfLandmarksTrackedAtFrame(state, frameIndex)
            pose_nid = find(state.ObservationGraph.Nodes.PoseId == frameIndex);
            N = degree(state.ObservationGraph, pose_nid);
        end
        
        function resetToPose(state, poseIdx)
            [nidx, ~, ~] = find(state.ObservationGraph.Nodes.PoseId > poseIdx ...
                & ~isfinite(state.ObservationGraph.Nodes.LandmarkId));
            
            state.ObservationGraph = rmnode(state.ObservationGraph, nidx);
            
            % TODO remove all glanding landamrks
        end
        
        function addLandmarksToPose(state, poseIdx, landmarksIdx, keypoints)
            % Take node id
            pose_nid = find(state.ObservationGraph.Nodes.PoseId == poseIdx);
            landmarks_nids = find(max(state.ObservationGraph.Nodes.LandmarkId == landmarksIdx.', [], 2));
            % Add edges between pose and landmarks
            nLandmarks = length(landmarksIdx);
            edgeTable = table(...
                [repmat(pose_nid, nLandmarks, 1) landmarks_nids], ...
                keypoints, 'VariableNames', {'EndNodes' 'Keypoints'});
            state.ObservationGraph = addedge(...
                state.ObservationGraph, edgeTable);
        end
        
        function addPose(state, poseIdx, R_CW, t_CW)
            % ADDPOSE Adds a new pose to the observation graph.
            % 
            % TODO DOCUMENT AGAIN!!
            %
            % state = state.ADDPOSE(pose, landmarksIndices, keypoints,
            % descriptors) adds a new pose to the observation graph. The
            % pose is described by the twist vector that embedds the
            % current transformation matrix of the i-th frame T_WC. When
            % adding a pose, the landmarks used to triangulate the pose are
            % required and this are provided through landmarksIndices (Nx1),
            % which is a subset of the ones obtained from state.getLandmarks.
            % keypoints (Nx2 [u v]) and descriptors (NxM) describe the
            % landmarks matched in the frame.
            % 
            % See also addLandmarks.
            
            newPose = table(poseIdx, Inf, 'VariableNames', {'PoseId', 'LandmarkId'});
            
            % Add pose node in observation graph
            state.ObservationGraph = addnode(state.ObservationGraph, newPose);
            
            % Add pose to pose table
            state.Poses(end+1,:) = table(poseIdx, ...
                R_CW(:).', t_CW(:).', reshape(-R_CW.' * t_CW, 1, []));
        end
        
        function [landmarks, landmarksIdx, mask] = addLandmarks(state, landmarks)
            % ADDLANDMARKS adds landmarks
            %
            % TODO document again! 3xN
            %
            % state = state.ADDLANDMARKS(landmarks) adds the given pose and
            % the landmarks (Nx3) provided in the observation graph, 
            % connecting them to the
            % newly add pose through the provided keypoints (Nx2 [u v]) and
            % descriptors (NxM).
            %
            % [state, landmarksIndices] = state.ADDLANDMARKS(...)
            % additionally returns the labels of the landamarks, to be used
            % for instance with addPose.
            %
            % See also addPose, getLandmarks.
            
            nLandmarks = size(landmarks, 2);
            
            % TODO eventually filter
            mask = ones(1, nLandmarks) > 0;
            
            landmarksIdx = (state.lastLandmark + 1 : state.lastLandmark + nLandmarks).';
            state.lastLandmark = state.lastLandmark + nLandmarks;
            
            state.Landmarks(end+1:end+nLandmarks,:) ...
                = table(landmarksIdx, landmarks.');
            
            newLandmarks = table(Inf(nLandmarks, 1), landmarksIdx,...
                'VariableNames', {'PoseId', 'LandmarkId'});
            % TODO inf does not work properly on the first column..
            state.ObservationGraph = addnode(state.ObservationGraph, newLandmarks);
        end
        
        function [landmarks, landmarksIdx, keypoints] ...
                = getObservations(state, poseId)
            % GETLANDMARKS
            % 
            % TODO COMMENT
            % 
            % See also addLandmarks, addPose.
            
            poseNode = find(state.ObservationGraph.Nodes.PoseId == poseId);
            [eid, landmarksNodesIdx] = outedges(state.ObservationGraph, poseNode);
            landmarksIdx = state.ObservationGraph.Nodes.LandmarkId(landmarksNodesIdx);
            landmarks = state.Landmarks.Position(landmarksIdx, :).';
            keypoints = state.ObservationGraph.Edges.Keypoints(eid, :).';
        end

        function addCandidates(state, ...
                frameIndex, unmatchedKeypoints, unmatchedDescriptors)
            % ADDCANDIDATES
            %
            % TODO COMMENT
            %
            % unmatchedKeypoints 2xN
            % unmatchedDescriptors MxN, M assumed to be always the same
            %
            % See also getCandidates, evaluateCandidates, pruneCandidates
            
            % Keep a window of state.candidatesWindowSize
            N = size(unmatchedKeypoints, 2);
            
            % Add candidates
            state.Candidates = [state.Candidates; 
                table(repmat(frameIndex, N, 1), ...
                    unmatchedKeypoints.', unmatchedDescriptors.', ...
                    'VariableNames', {'PoseId', 'Keypoints', 'Descriptors'})];
            
            verboseDisp(state.verbose, ...
                'Add %d candidates.\n', N);
        end

        function [keypoints, descriptors] = getCandidates(state, frameIdx)
            % GETCANDIDATES Returns candidates that can be used to
            % triangulate new landmarks.
            %
            % TODO update documentation!
            %
            % [candidateLabels, keypoints, descriptors] = state.GETCANDIDATES()
            %
            % candidateLabels is Nx1, the labels of the candidates. These
            % are needed to define which ones have been matched.
            %
            % keypoints is Nx2, the set of keypoints relative to the
            % candidateLabels [u v].
            %
            % descriptors is NxM, the set of descriptors relative to the
            % keypoints.
            % 
            % state.GETCANDIDATES(posesIndices) additionally specify the
            % set of poses indices to consider when selecting the
            % candidates to evaluate.
            %
            % See also evaluateCandidates, addCandidates, pruneCandidates
            
            [candidatePoseIdx, ~, ~] = find(state.Candidates.PoseId == frameIdx);
            
            keypoints = state.Candidates.Keypoints(candidatePoseIdx, :).';
            descriptors = state.Candidates.Descriptors(candidatePoseIdx, :).';
        end

        function evaluateCandidates(state, K, ...
                frameIndex, keypoints, ...
                candidateFrameIdx, matchesMask)
            % EVALUATECANDIDATES Updates the observation graph and the
            % candidates database given the found matches.
            %
            % TODO update DOCUMENTAION!! 2xN 2xN
            %
            % state = state.EVALUATECANDIDATES(frameIndex, matchLabels,
            % keypoints, descriptors, landmarks, K) Updates the observation 
            % graph and the candidates database given the found matches,
            % which are described by:
            %
            % frameIndex the index of the frame in which the candidates
            % have been matched again (e.g., the current frame).
            %
            % matchLabels (Nx1) the labels of the candidates (see getCandidates)
            % for which a match has been found.
            %
            % keypoints (Nx2, [u v]) and descriptors (NxM) are the 
            % descriptors of the matched candidates in the frameIndex-th 
            % image.
            % 
            % landmarks (Nx3, [X Y Z]) are the triangulated landmarks for
            % the matched candidates.
            
            candidateIdx = find(state.Candidates.PoseId == candidateFrameIdx);
            % extract from store
            candidateKeypoints = state.Candidates.Keypoints(candidateIdx, :).';
            % filter with mask
            matchedKeypoints = candidateKeypoints(:, matchesMask);
            N = size(matchedKeypoints, 2);
            
            % 1. Validation
            bearings1 = normalize(K \ [matchedKeypoints; ones(1, N)], 'norm');
            bearings2 = normalize(K \ [keypoints; ones(1, N)], 'norm');
            
            validMatches = reshape(...
                ... % evaluation metric
                abs(dot(bearings1, bearings2)) < state.cosineThreshold, ...
                [], 1);
            
            N = nnz(validMatches);
            if N == 0
                verboseDisp(state.verbose, ...
                    'Triangulated points have very low confidence.\n', []);
                return; % All the triangulations are invalid.
            end
            
            verboseDisp(state.verbose, ...
                'Triangulated %d points with enough confidence.\n', N);
            
            % Filter matches based on validMatches
            matchesMask(matchesMask > 0) = validMatches;

            % 2. Triangulate landmark
            [R_1W, t_1W] = state.getPose(candidateFrameIdx);
            [R_2W, t_2W] = state.getPose(frameIndex);

            T_1W = [R_1W, t_1W; 0, 0, 0, 1];
            T_2W = [R_2W, t_2W; 0, 0, 0, 1];
            T_21 = T_2W / T_1W;

            landmarks = triangulateFromPose(...
                [candidateKeypoints(1:2, matchesMask); ones(1, N)], ...
                [keypoints(1:2, validMatches); ones(1, N)], ...
                T_21, K, K, T_1W);

            [~, landmarksIdx, mask] = addLandmarks(state, landmarks(1:3, :));
            % update matches mask
            matchesMask(matchesMask > 0) = mask;
            validMatches(validMatches > 0) = mask;

            % 3. Add observation
            state.addLandmarksToPose(candidateFrameIdx, ...
                landmarksIdx, candidateKeypoints(:,matchesMask).');
            state.addLandmarksToPose(frameIndex, ...
                landmarksIdx, keypoints(1:2,validMatches).');

            if state.verbose
                N = state.getNumberOfLandmarksTrackedAtFrame(frameIndex);
                verboseDisp(state.verbose, ...
                    'Tracking %d landmarks at frame %d.\n', ...
                    [N, frameIndex]);
            end

            % 4. Remove triangulated candidates
            % We directly replace the old candidates with the remaining
            % ones.
            candidateDescriptors ...
                = state.Candidates.Descriptors(candidateIdx, :).';
            state.Candidates(candidateIdx, :) = []; % Remove everything

            state.addCandidates(candidateFrameIdx, ...
                candidateKeypoints(:, matchesMask), ...
                candidateDescriptors(:, matchesMask));
        end

        function resetCandidates(state, minPoseIndex)
            % PRUNE Prune the candidates table.
            % 
            % TODO update documentation
            %
            % state = state.pruneCandidates(minPoseIndex) remove all 
            % candidates from poses associated to an index less or equal to
            % minPoseIndex.
            %
            % See also addCandidates, getCandidates, evaluateCandidates
            % 
            % TODO maxDistance based pruning
            
            % Candidates
            state.Candidates(state.Candidates.PoseId < minPoseIndex, :) = [];
        end
%
%         function state = prune(state, minPoseIndex)
%             % PRUNE Prune the observation graph.
%             % 
%             % state = state.prune(minPoseIndex) remove all poses with an 
%             % index less or equal to minPoseIndex. Removes also all
%             % landmarks that will remain unobserved, i.e., such that there
%             % are no poses stored in which the landmarks have been obseved,
%             % i.e., nodes with zero degree in the observation graph.
%             % 
%             % TODO maxDistance based pruning
%             % TODO save on file before pruning
%             
%             % Remove poses
%             posesToRemove = state.Poses.Id(state.Poses.Id < minPoseIndex);
%             [poseNodesToRemove, ~, ~] = find(state.ObservationGraph.Nodes.PoseId == posesToRemove.');
%             state.ObservationGraph = rmnode(state.ObservationGraph, poseNodesToRemove);
%             [poseIdx, ~, ~] = find(state.Poses.Id == posesToRemove.');
%             state.Poses(poseIdx, :) = [];
%             
%             % Remove Landmarks not observed by any pose
%             [landmarkNodesToRemove, ~, ~] = find(isfinite(state.ObservationGraph.Nodes.LandmarkId));
%             % No observations
%             landmarkNodesToRemove = landmarkNodesToRemove(...
%                 0 == degree(state.ObservationGraph, landmarkNodesToRemove));
%             landmarksToRemove = state.ObservationGraph.Nodes.LandmarkId(landmarkNodesToRemove);
%             state.ObservationGraph = rmnode(state.ObservationGraph, landmarkNodesToRemove);
%             [landmarksIdx, ~, ~] = find(state.Landmarks.Id == landmarksToRemove.');
%             state.Landmarks(landmarksIdx, :) = [];
%         end
%         
        function [hiddenState, observations, bundleIdx] ...
                = getOptimizationDS(state, bundleSize)
            % GETBUNDLEADJUSTMENTDS Returns a convenient data structure to
            % perform bundle adjustment.
            %
            % TODO update documentation and allow to slide window
            % 
            % [hiddenState, observations] = state.getBundleAdjustmentDS()
            % returns a convenient data structure to perform bundle
            % adjustment from all the currently stored poses.
            %
            % hiddenState is [tau_1.', ..., tau_n.', P_1.', ..., P_m.'],
            % where tau_i is the twist vector, describing the i-th camera 
            % pose and P_i is the 3D position of the i-th landmark.
            %
            % observations is [n, m, O_1.', ..., O_n.'], where n is the
            % number of frames, m is the number of landmarks and O_i is 
            % [k_i, p_i1.', ..., p_ik_i.', li1, ..., lik_i], where k_i is
            % the number landmarks observed at the i-th frame, p_ij is the
            % j-th keypoints found in the i-th frame and lij is the index
            % of the landmark corresponding to the j-th keypoint of the
            % i-th frame.
            %
            % state.getBundleAdjustmentDS(poseIndices) additionally allows
            % to specify which poses to consider when building the
            % data structure.
            
            arguments
                state
                bundleSize = Inf
            end
            
            nPoses = size(state.Poses, 1);
            bundleSize = min(bundleSize, nPoses);
            poseIdx = nPoses-bundleSize+1:nPoses;
            frameIdx = state.Poses.Id(poseIdx);
            
            hiddenState = [];
            observations = [];
            
            [posenids, ~] = find(state.ObservationGraph.Nodes.PoseId == frameIdx.');
            landmarksIdx = [];
            for i = 1:bundleSize
                R_CW = reshape(state.Poses.R_CW(poseIdx(i), :), 3, 3);
                t_WC = reshape(state.Poses.Position(poseIdx(i), :), 3, 1);
                
                T_WC = [R_CW.', t_WC; 0, 0, 0, 1];
                
                tau_i = homogMatrix2twist(T_WC);
                hiddenState = [hiddenState; tau_i];
                
                [leids, lnids] = outedges(state.ObservationGraph, posenids(i));
                landmarksIdx_i = state.ObservationGraph.Nodes.LandmarkId(lnids);
                landmarksIdx = [landmarksIdx; landmarksIdx_i];
                keypoints = state.ObservationGraph.Edges.Keypoints(leids, :).';
                
                observations = [observations; numel(lnids); 
                    reshape(keypoints, [], 1); landmarksIdx_i];
            end
            
            % So that all observations points to the same landmarks
            landmarksIdx = unique(landmarksIdx);
            j = 1;
            for i = 1:bundleSize
                k_i = observations(j);
                landmarksIdx_i = observations(j+2*k_i+1: j+3*k_i);
                [l_i, ~] = find(landmarksIdx == landmarksIdx_i.');
                observations(j+2*k_i+1: j+3*k_i) = l_i;
                
                j = j+3*k_i+1;
            end
            
            landmarks = state.Landmarks.Position(landmarksIdx, :).';
            
            hiddenState = [hiddenState; reshape(landmarks, [], 1)];
            observations = [bundleSize; size(landmarks, 2); observations];
            
            bundleIdx = [numel(poseIdx); 
                poseIdx.'; landmarksIdx];
        end
        
        function optimizedBundle(state, hiddenState, bundleIdx)
            nPoses = bundleIdx(1);
            poseIdx = bundleIdx(2:2+nPoses-1);
            landmarksIdx = bundleIdx(2+nPoses:end);
            
            for i = 1:nPoses
                tau_i = hiddenState(6*(i-1)+1:6*i);
                T_WC = twist2HomogMatrix(tau_i);
                R_CW = T_WC(1:3,1:3).';
                t_WC = T_WC(1:3,end);
                t_CW = -R_CW * t_WC;
                
                state.Poses.R_CW(poseIdx(i), :) = reshape(R_CW, 1, []);
                state.Poses.t_CW(poseIdx(i), :) = reshape(t_CW, 1, []);
                state.Poses.Position(poseIdx(i), :) = reshape(t_WC, 1, []);
            end
            
            landmarks = reshape(hiddenState(6*nPoses+1:end), 3, []);
            state.Landmarks.Position(landmarksIdx, :) = landmarks.';
        end
    end
end

