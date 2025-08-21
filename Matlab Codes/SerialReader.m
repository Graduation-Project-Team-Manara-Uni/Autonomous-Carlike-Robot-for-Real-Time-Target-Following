% Clear workspace, command window, and close any open serial ports
clear;
clc;
if ~isempty(instrfind)
     fclose(instrfind);
     delete(instrfind);
end

% --- Setup the Serial Port ---
% Replace 'COM30' with the correct COM port for your Arduino
comPort = 'COM30'; 
baudRate = 9600;

% Create a serial object
arduino = serial(comPort, 'BaudRate', baudRate);

% Set the terminator to 'LF' (Line Feed), which matches Arduino's println()
set(arduino, 'Terminator', 'LF');

try
    % Open the serial port connection
    fopen(arduino);
    
    disp('Connection opened. Waiting for data...');

    % --- Read Data in a Loop ---
    for i = 1:10 % Read 10 arrays
        voltstr = fgetl(arduino);
        if ischar(voltstr) && ~isempty(voltstr)
            dataCell = strsplit(strtrim(voltstr), ',');
            Volts = str2double(dataCell);
            disp('Volt Array:');disp(Volts);
        else
            disp('Warning: Empty Volt string received.');
        end
        
        omegastr = fgetl(arduino);
        if ischar(omegastr) && ~isempty(omegastr)
            dataCell = strsplit(strtrim(omegastr), ',');
            Omegas = str2double(dataCell);
            disp('Omega Array:');disp(Omegas);
        else
            disp('Warning: Empty Omega string received.');
        end
        
    end

    % --- Clean Up ---
    % Close and delete the serial object
    fclose(arduino);
    delete(arduino);
    clear arduino;
    disp('Connection closed.');

catch ME
    % In case of an error, close the port and display the error message
    disp('An error occurred:');
    disp(ME.message);
    
    % Clean up resources
    if ~isempty(instrfind)
         fclose(instrfind);
         delete(instrfind);
    end
end