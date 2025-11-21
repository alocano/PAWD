% Run Proximity Plot - Starter Script
clear; clc; close all;

fprintf('=== VCNL4020 Proximity Data Plotter ===\n');
fprintf('Make sure your STM32 NUCLEO-F767ZI is:\n');
fprintf('1. Connected via USB\n');
fprintf('2. Programmed with the proximity sensor code\n');
fprintf('3. The sensor is properly connected\n\n');



try
    proximity();
catch ME
    fprintf('Error: %s\n', ME.message);
    fprintf('\nTroubleshooting:\n');
    fprintf('1. Check COM port in Device Manager\n');
    fprintf('2. Verify STM32 is programmed correctly\n');
    fprintf('3. Ensure no other program is using the serial port\n');
    fprintf('4. Check baud rate matches (115200)\n');
end

fprintf('\nProgram finished.\n');