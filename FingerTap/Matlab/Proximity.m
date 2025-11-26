clear; clc; close all;

% Create serial connection to STM32
s = serialport('COM5',115200); % Replace COM3 with your actual port
fopen(s);

% Initialize variables
maxInterrupts = 10;
interruptCount = 0;
timeData = [];
proximityData = [];
interruptTimes = [];
interruptProximity = [];
startTime = tic;

% Configure plot
figure;
hold on;
hPlot = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Proximity Data');
hInterrupts = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'GPIO Interrupts');
xlabel('Time (s)');
ylabel('Proximity');
title('VCNL4020 Proximity Data vs Time with GPIO Interrupts');
grid on;
legend('show');

fprintf('Starting data acquisition...\n');
fprintf('Waiting for GPIO interrupts...\n');

% Variables to track interrupt state
previousInterruptCount = 0;

try
    while interruptCount < maxInterrupts
        % Check for available data
        if s.BytesAvailable > 0
            % Read data from serial
            dataLine = fgetl(s);
            
            % Parse the data (assuming format: "proximity,interrupt_count")
            dataValues = sscanf(dataLine, '%d,%d');
            
            if length(dataValues) >= 2
                proximity = dataValues(1);
                interruptCount = dataValues(2);
                
                % Calculate elapsed time
                currentTime = toc(startTime);
                
                % Store data
                timeData(end+1) = currentTime;
                proximityData(end+1) = proximity;
                
                % Check if new interrupt occurred
                if interruptCount > previousInterruptCount
                    % Store interrupt data point
                    interruptTimes(end+1) = currentTime;
                    interruptProximity(end+1) = proximity;
                    
                    fprintf('>>> GPIO INTERRUPT #%d detected at %.2fs <<<\n', ...
                        interruptCount, currentTime);
                    
                    % Update interrupt markers
                    set(hInterrupts, 'XData', interruptTimes, 'YData', interruptProximity);
                end
                
                previousInterruptCount = interruptCount;
                
                % Update main plot
                set(hPlot, 'XData', timeData, 'YData', proximityData);
                
                % Adjust x-axis to show all data with some padding
                if length(timeData) > 1
                    xlim([0 max(timeData) + 1]);
                end
                
                % Adjust y-axis with some margin
                if length(proximityData) > 1
                    ymin = min(proximityData);
                    ymax = max(proximityData);
                    yrange = ymax - ymin;
                    if yrange == 0
                        yrange = 100; % Default range if no variation
                    end
                    ylim([ymin - 0.1*yrange, ymax + 0.1*yrange]);
                end
                
                drawnow;
                
                % Display current status
                fprintf('Time: %.2fs, Proximity: %d, Interrupts: %d/%d\n', ...
                    currentTime, proximity, interruptCount, maxInterrupts);
            end
        end
        
        % Small pause to prevent CPU overload
        pause(0.01);
    end
    
    fprintf('\n=== Data acquisition complete! ===\n');
    fprintf('Received %d interrupts at times: ', maxInterrupts);
    fprintf('%.2f ', interruptTimes);
    fprintf('seconds\n');
    
    % Create a summary plot with final annotations
    figure;
    hold on;
    plot(timeData, proximityData, 'b-', 'LineWidth', 2, 'DisplayName', 'Proximity Data');
    plot(interruptTimes, interruptProximity, 'ro', 'MarkerSize', 10, ...
         'LineWidth', 2, 'DisplayName', 'GPIO Interrupts');
    
    % Annotate each interrupt point
    for i = 1:length(interruptTimes)
        text(interruptTimes(i), interruptProximity(i), ...
             sprintf('  Int #%d', i), ...
             'VerticalAlignment', 'bottom', 'FontSize', 10, ...
             'FontWeight', 'bold', 'Color', 'red');
    end
    
    xlabel('Time (s)');
    ylabel('Proximity');
    title('Final Plot: VCNL4020 Proximity Data with GPIO Interrupt Markers');
    grid on;
    legend('show');
    
    % Save data to file with interrupt information
    dataTable = table(timeData', proximityData', 'VariableNames', {'Time_s', 'Proximity'});
    writetable(dataTable, 'proximity_data.csv');
    
    % Save interrupt times separately
    interruptTable = table((1:length(interruptTimes))', interruptTimes', interruptProximity', ...
        'VariableNames', {'Interrupt_Number', 'Time_s', 'Proximity_Value'});
    writetable(interruptTable, 'interrupt_times.csv');
    
    fprintf('Data saved to proximity_data.csv\n');
    fprintf('Interrupt times saved to interrupt_times.csv\n');
    
    % =====================================================================
    % FINGER TAPPING TEST ANALYSIS
    % =====================================================================
    fprintf('\n\n=== PERFORMING FINGER TAPPING TEST ANALYSIS ===\n');
    
    [patientScore, analysisResults] = analyzeFingerTapping(...
        proximityData, timeData, interruptTimes, interruptProximity);
    
    % Create comprehensive analysis report figure
    figure('Position', [100, 100, 1200, 800]);
    
    % Plot 1: Raw data with taps
    subplot(2,2,1);
    plot(timeData, proximityData, 'b-', 'LineWidth', 1.5);
    hold on;
    if length(interruptTimes) >= 10
        plot(interruptTimes(1:10), interruptProximity(1:10), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    else
        plot(interruptTimes, interruptProximity, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    end
    xlabel('Time (s)');
    ylabel('Proximity');
    title('Finger Tapping Test - Raw Data');
    legend('Proximity', 'Taps', 'Location', 'best');
    grid on;
    
    % Plot 2: Rhythm analysis (inter-tap intervals)
    subplot(2,2,2);
    if length(interruptTimes) >= 10
        plot(1:9, diff(interruptTimes(1:10)), 'g-o', 'LineWidth', 2, 'MarkerSize', 6);
        xlabel('Tap Number');
        ylabel('Inter-Tap Interval (s)');
        title('Rhythm Analysis');
        grid on;
    else
        text(0.3, 0.5, 'Insufficient taps for rhythm analysis', 'FontSize', 12);
        axis off;
    end
    
    % Plot 3: Amplitude progression
    subplot(2,2,3);
    if length(interruptTimes) >= 10
        plot(1:10, interruptProximity(1:10), 'm-s', 'LineWidth', 2, 'MarkerSize', 6);
        xlabel('Tap Number');
        ylabel('Amplitude (Proximity)');
        title('Amplitude Progression');
        grid on;
    else
        text(0.3, 0.5, 'Insufficient taps for amplitude analysis', 'FontSize', 12);
        axis off;
    end
    
    % Plot 4: Test summary
    subplot(2,2,4);
    scoreDescriptions = {
        '0: NORMAL - No problems detected';
        '1: SLIGHT - Minor interruptions, slight slowing, or end-of-sequence amplitude reduction';
        '2: MILD - 3-5 interruptions, mild slowing, or mid-sequence amplitude reduction';
        '3: MODERATE - >5 interruptions, moderate slowing, or early amplitude reduction';
        '4: SEVERE - Cannot or barely perform task due to severe issues'
    };
    
    summaryText = sprintf('FINGER TAPPING TEST SUMMARY\n\nFinal Score: %d\n\n%s', ...
        patientScore, scoreDescriptions{patientScore+1});
    text(0.05, 0.7, summaryText, 'FontSize', 12, 'FontWeight', 'bold', ...
        'VerticalAlignment', 'top');
    axis off;
    title('Test Summary');
    
    sgtitle('Parkinson''s Disease Finger Tapping Test Analysis', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save analysis results
    save('finger_tapping_analysis.mat', 'patientScore', 'analysisResults');
    fprintf('\nAnalysis results saved to finger_tapping_analysis.mat\n');
    
catch ME
    fprintf('Error: %s\n', ME.message);
end

% Clean up
fclose(s);
delete(s);
clear s;

% =========================================================================
% FINGER TAPPING ANALYSIS FUNCTION
% =========================================================================
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
        analysisResults.scoreDescription = 'SEVERE: Cannot complete 10 taps';
        analysisResults.numTaps = length(interruptTimes);
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
    
    % Store score description
    scoreDescriptions = {
        'NORMAL - No problems detected';
        'SLIGHT - Minor interruptions, slight slowing, or end-of-sequence amplitude reduction';
        'MILD - 3-5 interruptions, mild slowing, or mid-sequence amplitude reduction';
        'MODERATE - >5 interruptions, moderate slowing, or early amplitude reduction';
        'SEVERE - Cannot or barely perform task due to severe issues'
    };
    analysisResults.scoreDescription = scoreDescriptions{score+1};
    
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
    if metrics.meanInterval > 0
        metrics.consistencyScore = 1 - (metrics.intervalVariability/metrics.meanInterval);
    else
        metrics.consistencyScore = 0;
    end
    metrics.numTaps = numTaps;
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
    fprintf('Number of taps analyzed: %d\n', metrics.numTaps);
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