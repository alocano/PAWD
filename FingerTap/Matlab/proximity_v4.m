clear; clc; close all;

% Create serial connection to STM32
s = serialport('COM5',115200); % Replace COM3 with your actual port


% Initialize variables
maxInterrupts = 10;
interruptCount = 0;
timeData = [];
proximityData = [];
interruptTimes = [];
interruptProximity = [];
%startTime = tic;

% Configure plot
figure;
hold on;
hPlot = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Proximity Data');
hInterrupts = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'GPIO Interrupts');
xlabel('Time (s)');
ylabel('Proximity');
title(sprintf('Proximity Data - Interrupt Count: %d', interruptCount));
grid on;
legend('show');

fprintf('Starting data acquisition...\n');
fprintf('Waiting for GPIO interrupts...\n');

% Variables to track interrupt state
previousInterruptCount = 0;

try
    while interruptCount < maxInterrupts
        % Check for available data
        if s.NumBytesAvailable > 0
            % Read data from serial
            dataLine = readline(s);
            
            % Parse the data (assuming format: "proximity, *counter, elapsedTime, elapsedTimeInt")
            dataValues = sscanf(dataLine, '%d,%d,%d,%d');
            
            if length(dataValues) >= 4
                proximity = dataValues(1);
                interruptCount = dataValues(2);
                
                % Calculate elapsed time
                currentTime = dataValues(3);

                
                
               
                
                % Store data
                timeData(end+1) = currentTime;
                proximityData(end+1) = proximity;
                
                if ~isempty(timeData) && ~isempty(proximityData)
                    % Check if new interrupt occurred
                    if interruptCount > previousInterruptCount
                        % Store interrupt data point
                        interruptTimes(end+1) = dataValues(4);
                        interruptProximity(end+1) = proximity;
                        
                        fprintf('>>> GPIO INTERRUPT #%d detected at %.2fs <<<\n', ...
                            interruptCount, currentTime);
                        
                        % Update interrupt markers (only if we have data)
                        if ~isempty(interruptTimes)
                            set(hInterrupts, 'XData', interruptTimes, 'YData', interruptProximity);
                        end
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
                
                try
                    drawnow;
                catch ME
                        fprintf('Drawnow error: %s\n', ME.message);
                end
                
                % Display current status
                fprintf('Time: %.2fs, Proximity: %d, Interrupts: %d/%d\n', ...
                    currentTime, proximity, interruptCount, maxInterrupts);
                end
            end
        end
        
        % Small pause to prevent CPU overload
        %pause(0.01);
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
    
catch ME
    fprintf('Error: %s\n', ME.message);
end

% Clean up
clear s;