function proximity()
    % Modern MATLAB serialport implementation
    com_port = 'COM6';  % Change to your COM port
    baud_rate = 115200;
    
    % Clear existing connections
    if ~isempty(serialportfind)
        clear instrfind;
    end
    
    try
        % Create serialport object
        s = serialport(com_port, baud_rate);
        configureTerminator(s, "LF");
    catch ME
        error('Failed to open serial port: %s', ME.message);
    end

    % Initialize variables
    proximity_data = [];
    interrupt_data = [];
    sample_count = 0;

    % Create figure
    fig = figure('Name', 'VCNL4020 Proximity Data', 'NumberTitle', 'off');
    ax = axes('Parent', fig);
    
    fprintf('Starting data acquisition...\n');
    
    while true
        if s.NumBytesAvailable > 0
            try
                data_line = readline(s);
                
                if contains(data_line, 'END')
                    fprintf('Acquisition complete.\n');
                    break;
                end
                
                % Parse data
                parsed_data = sscanf(data_line, '%d,%d,%d');
                
                if length(parsed_data) == 3
                    sample_count = sample_count + 1;
                    proximity_data(sample_count) = parsed_data(1);
                    interrupt_data(sample_count) = parsed_data(2);
                    
                    % Update plot
                    cla(ax);
                    plot(ax, 1:sample_count, proximity_data, 'b-', 'LineWidth', 1.5);
                    hold(ax, 'on');
                    
                    % Mark interrupt points
                    interrupt_points = find(diff([0 interrupt_data]) > 0);
                    plot(ax, interrupt_points, proximity_data(interrupt_points), ...
                         'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
                    hold(ax, 'off');
                    
                    title(ax, sprintf('Proximity Data - Interrupt Count: %d', interrupt_data(end)));
                    xlabel(ax, 'Sample Index');
                    ylabel(ax, 'Proximity Value');
                    grid(ax, 'on');
                    legend(ax, 'Proximity', 'Interrupt Events', 'Location', 'best');
                    
                    drawnow;
                end
            catch
                % Continue on read errors
            end
        end
        pause(0.01);
    end
    
    clear s;
end