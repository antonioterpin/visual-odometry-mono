function [state] = triangulateNewData(state, K, detector, nSkip, ...
    frameIdx, unmatchedKeypoints, unmatchedDescriptors, verbose)
%TRIANGULATION Summary of this function goes here
%   Detailed explanation goes here

    verboseDisp(verbose, 'Triangulation\n---------------------\n', []);

    j = state.candidatesWindowSize;
    while j >= 1 && ~isempty(unmatchedKeypoints)
        candidateFrameIdx = frameIdx - j * nSkip; 
        
        % 1. Get candidates from candidateFrameIdx
        [cKeypoints, cDescriptors] = state.getCandidates(candidateFrameIdx);
        
        if ~isempty(cKeypoints)
            verboseDisp(verbose, ...
                'Triangulation with frame %d...\n', candidateFrameIdx);
            
            % 2. Match
            [matches, ~, matchedKeypoints] = detector.getMatches(...
                cDescriptors, unmatchedDescriptors, cKeypoints, unmatchedKeypoints);
            unmatchedKeypoints = unmatchedKeypoints(:, matches == 0);
            unmatchedDescriptors = unmatchedDescriptors(:, matches == 0);
            matchesMask = zeros(1, size(cKeypoints, 2));
            matchesMask(matches(matches > 0)) = 1;

            verboseDisp(verbose, ...
                'Found %d matches.\n', size(matchedKeypoints, 2));

            % 3. Evaluate matched candidates
            state.evaluateCandidates(K, frameIdx, matchedKeypoints, ...
                    candidateFrameIdx, matchesMask > 0);
        end
        
        j = j - 1;
    end

    % Keep still unmatched keypoints as candidates for future
    % frames.
    % TODO some filtering can be introduced here
    state.addCandidates(frameIdx, ...
        unmatchedKeypoints, unmatchedDescriptors);
    
    verboseDisp(verbose, '=====================\n', []);
end

