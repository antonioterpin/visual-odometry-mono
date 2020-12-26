function [model, inliers] = RANSAC(modelFromSample, errorMetric, ...
    nParams, data, nIterations, otherParams, verbose, confidence)
% RANSAC Generic RANSAC implementation.
% 
% [model, inliers] = RANSAC(modelFromSample, errorMetric, nParams, data)
% returns the fitted model and the logical indexing of the inliers in the
% data with respect to the fitted model. Each candidate model is fitted
% using the modelFromSample function provided and a sample of nParams
% element of the data. errorMetric is a function that is used to evaluate
% the candidate model with respect to all the data and determine the
% corresponding inliers. The best model (in the sense of most likelyhood,
% i.e., with the largest amount of inliers) is kept.
% modelFromSample is a function that takes as parameters a sample of the data,
% in the same format provided to the RANSAC function, and otherParams,
% which are forwarded directly as provided to RANSAC (see next signatures).
% errorMetric is a function that takes as parameters a candidate model, as
% returned from modelFromSample, all the data as provided to RANSAC and
% otherParams, as provided to RANSAC.
% data is a K-dimensional matrix. Samples are taken from the second
% dimension without repetition.
%
% RANSAC(..., nIterations) additionally allows to specify the number of 
% iterations to perform when looking for the best model.
% Default is nIterations = 1000.
%
% RANSAC(..., otherParams) if modelFromSample and errorMetric requires
% extra parameters other then data and candidate (which is fitted using
% modelFromSample), these can specified in the otherParams object. It is up
% to the users to choose the format that best suits them. RANSAC will only
% forward this parameter to both the functions.
%
% RANSAC(..., verbose) additionally allows to specify whether RANSAC has to
% display its progress (verbose == true) or not (verbose == false). 
% Default is verbose == false.
%
% Example:
% p0, p1 are 3xN homogenous coordinates in the image1 and image2
% respectively. fundamentalEightPoint is the model we are fitting, using 8
% samples of the data, which is p0 stacked over p1 (data = [p0; p1]).
% The iterations are limited at 2000. No extra parameters are required.
% The model is verified through errorMetric, i.e., all the data are
% evaluated through the candidate and the amount of inliers with respect to
% the fitted model is calculated. At the end of the iterations, the best
% model is given, with a logical indexing vector that describes which
% elements of the data provided are to be considered inliers.
%
% [F, inliers] = RANSAC(...
%     @(sample, otherParams) fundamentalEightPoint(sample(1:3,:), sample(4:6,:)), ...
%     @(candidate, data, otherParams) errorMetric(data(1:3,:), data(4:6,:), candidate(:)),...
%     8, [p0; p1], 2000, []);

arguments
    modelFromSample function_handle
    errorMetric function_handle
    nParams {mustBeNumeric(nParams)}
    data {mustContainEnoughData(data,nParams)}
    nIterations (1,1) = 1000
    otherParams = []
    verbose logical = false
    confidence = 0.95
end

verboseDisp(verbose, 'Starting RANSAC..');

maxNumberOfInliers = -1;

if confidence > 0
    assert(confidence < 1, 'confidence must be between 0 and 1.');
    nIterations = Inf;
end

it = 1;
while it <= nIterations
    sample = datasample(data, nParams, 2, 'Replace', false);

    candidate = modelFromSample(sample, otherParams);
    inliers_ = errorMetric(candidate, data, otherParams);
    nInliers = nnz(inliers_);

    if nInliers > maxNumberOfInliers
        model = candidate;
        maxNumberOfInliers = nInliers;
        inliers = inliers_;
        if confidence > 0
            % outlier ratio estimate
            outlierRatio = min(0.9, 1 - maxNumberOfInliers / numel(inliers));
            nIterations = log(1-confidence) / log(1-(1-outlierRatio)^nParams);
            % cap the number of iterations at 15000
            nIterations = min(15000, nIterations);
    %         verboseDisp(verbose, ...
    %             '   estimated outlierRatio %.3f, %d it.\n', ...
    %             [outlierRatio * 100, nIterations]);
        end
    end

%     verboseDisp(verbose, [getProgressString(it, nIterations) ...
%         'Iteration %d) Found %d inliers, max is %d.\n'], ...
%         [it, nInliers, maxNumberOfInliers]);
    it = it + 1;
end

verboseDisp(verbose, ...
    'RANSAC ended with %d inliers after %d iterations.\n', ...
    [maxNumberOfInliers, it - 1]);
end

function mustContainEnoughData(data,nParams)
    if size(data,2) < nParams
        eid = 'Size:notEnoughData';
        msg = 'Size of dataset must be at least the nParams required for the model fitting.';
        throwAsCaller(MException(eid,msg))
    end
end
