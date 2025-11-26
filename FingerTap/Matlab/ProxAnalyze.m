function [score, analysisResults] = analyzeFingerTapping(proximityData, timeData, interruptTimes, interruptProximity)
%ANALYZEFINGERTAPPING Analyzes finger tapping test data for Parkinson's assessment
%   Inputs:
%   - proximityData: Array of proximity values over time
%   - timeData: Array of time points corresponding to proximityData
%   - interruptTimes: Times when GPIO interrupts occurred (taps)
%   - interruptProximity: Proximity values at interrupt times
%
%   Outputs:
%   - score: Finger tapping test score (0-4)
%   - analysisResults: Structure with detailed analysis data

    fprintf('\n=== FINGER TAPPING TEST ANALYSIS ===\n');
    
    % Check if we have enough interrupts (taps)
    if length(interruptTimes) < 10
        fprintf('Warning: Only %d taps detected. Test requires 10 taps.\n', length(interruptTimes));
        score = 4; % Severe if cannot complete 10 taps
        analysisResults = struct();
        return;
    end

    % Extract the first 10 taps for analysis
    numTaps = min(10, length(interruptTimes));
    tapTimes = interruptTimes(1:numTaps);
    tapProximity = interruptProximity(1:numTaps);
    
    % Calculate key metrics
    analysisResults = calculateMetrics(proximityData, timeData, tapTimes, tapProximity);
    
    % Determine score based on criteria
    score = determineScore(analysisResults);
    
    % Display results
    displayResults(analysisResults, score);
end

function metrics = calculateMetrics(proximityData, timeData, tapTimes, tapProximity)
% Calculate all relevant metrics for scoring
    
    numTaps = length(tapTimes);
    
    % 1. Speed analysis (inter-tap intervals)
    interTapIntervals = diff(tapTimes);
    metrics.meanInterval = mean(interTapIntervals);
    metrics.intervalVariability = std(interTapIntervals);
    
    % 2. Rhythm interruptions/hesitations
    % Find pauses longer than expected
    expectedInterval = metrics.meanInterval;
    hesitationThreshold = 1.5 * expectedInterval; % 50% longer than average
    metrics.hesitationCount = sum(interTapIntervals > hesitationThreshold);
    
    % 3. Amplitude analysis
    metrics.amplitudeData = tapProximity;
    metrics.meanAmplitude = mean(tapProximity);
    metrics.amplitudeStd = std(tapProximity);
    
    % 4. Amplitude decrement analysis
    % Calculate amplitude reduction over the sequence
    if numTaps >= 3
        firstThird = mean(tapProximity(1:min(3, numTaps)));
        middleThird = mean(tapProximity(ceil(numTaps/3):floor(2*numTaps/3)));
        lastThird = mean(tapProximity(max(1, numTaps-2):numTaps));
        
        metrics.amplitudeReduction = [(firstThird - middleThird)/firstThird * 100, ...
                                     (firstThird - lastThird)/firstThird * 100];
    else
        metrics.amplitudeReduction = [0, 0];
    end
    
    % 5. Detect freezing/halts (very long pauses)
    freezeThreshold = 3.0 * expectedInterval; % 3x longer than average
    metrics.freezeCount = sum(interTapIntervals > freezeThreshold);
    
    % 6. Overall movement quality
    metrics.totalTime = tapTimes(end) - tapTimes(1);
    metrics.consistencyScore = 1 - (metrics.intervalVariability/metrics.meanInterval);
end

function score = determineScore(metrics)
% Determine finger tapping score based on calculated metrics
    
    score = 0; % Start with normal
    
    % Check interruptions/hesitations
    if metrics.hesitationCount >= 6 || metrics.freezeCount >= 1
        score = max(score, 3); % Moderate
    elseif metrics.hesitationCount >= 3
        score = max(score, 2); % Mild
    elseif metrics.hesitationCount >= 1
        score = max(score, 1); % Slight
    end
    
    % Check amplitude decrement
    amplitudeReduction = max(metrics.amplitudeReduction);
    if amplitudeReduction > 50 % Severe reduction
        score = max(score, 3);
    elseif amplitudeReduction > 30 % Moderate reduction
        score = max(score, 2);
    elseif amplitudeReduction > 15 % Mild reduction
        score = max(score, 1);
    end
    
    % Check speed (total time for 10 taps)
    if metrics.totalTime > 8.0 % Very slow
        score = max(score, 3);
    elseif metrics.totalTime > 5.0 % Moderately slow
        score = max(score, 2);
    elseif metrics.totalTime > 3.0 % Slightly slow
        score = max(score, 1);
    end
    
    % Severe category - if multiple severe issues or cannot complete properly
    if (metrics.hesitationCount >= 8 && amplitudeReduction > 40) || metrics.freezeCount >= 2
        score = 4;
    end
end

function displayResults(metrics, score)
% Display comprehensive analysis results
    
    fprintf('\n--- ANALYSIS RESULTS ---\n');
    fprintf('Number of taps analyzed: %d\n', length(metrics.amplitudeData));
    fprintf('Total test duration: %.2f seconds\n', metrics.totalTime);
    fprintf('Average inter-tap interval: %.3f seconds\n', metrics.meanInterval);
    fprintf('Rhythm consistency: %.2f (0-1 scale, higher=better)\n', metrics.consistencyScore);
    
    fprintf('\n--- INTERRUPTIONS/HESITATIONS ---\n');
    fprintf('Number of hesitations: %d\n', metrics.hesitationCount);
    fprintf('Number of freezing episodes: %d\n', metrics.freezeCount);
    
    fprintf('\n--- AMPLITUDE ANALYSIS ---\n');
    fprintf('Average amplitude: %.2f\n', metrics.meanAmplitude);
    fprintf('Amplitude variability: %.2f\n', metrics.amplitudeStd);
    fprintf('Amplitude reduction: %.1f%% (early), %.1f%% (late)\n', ...
        metrics.amplitudeReduction(1), metrics.amplitudeReduction(2));
    
    fprintf('\n--- SCORING ---\n');
    scoreDescriptions = {
        '0: NORMAL - No problems detected';
        '1: SLIGHT - Minor interruptions, slight slowing, or end-of-sequence amplitude reduction';
        '2: MILD - 3-5 interruptions, mild slowing, or mid-sequence amplitude reduction';
        '3: MODERATE - >5 interruptions, moderate slowing, or early amplitude reduction';
        '4: SEVERE - Cannot or barely perform task due to severe issues'
    };
    
    fprintf('FINGER TAPPING SCORE: %d\n', score);
    fprintf('%s\n', scoreDescriptions{score+1});
    
    % Provide recommendations
    fprintf('\n--- RECOMMENDATIONS ---\n');
    switch score
        case 0
            fprintf('✓ Performance within normal limits\n');
        case 1
            fprintf('✓ Monitor for progression\n');
            fprintf('✓ Consider mild intervention if symptoms worsen\n');
        case 2
            fprintf('✓ Recommend follow-up assessment\n');
            fprintf('✓ Consider occupational therapy evaluation\n');
        case 3
            fprintf('✓ Recommend neurological evaluation\n');
            fprintf('✓ Consider medication adjustment if on treatment\n');
        case 4
            fprintf('✓ Urgent neurological evaluation recommended\n');
            fprintf('✓ Significant functional impairment likely\n');
    end
end