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
fclose(s);
delete(s);
clear s;