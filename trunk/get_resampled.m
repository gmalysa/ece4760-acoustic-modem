try
    serial = serial('COM4', 'BaudRate', 230400, 'InputBufferSize', 1024, 'Terminator', '');
catch exception
    delete(serial);
    clear serial;
    serial = serial('COM4', 'BaudRate', 230400, 'InputBufferSize', 1024, 'Terminator', '');
end
try
    fopen(serial);
catch exception
    delete(serial);
    clear serial;
    serial = serial('COM4', 'BaudRate', 230400, 'InputBufferSize', 1024, 'Terminator', '');
    fopen(serial);
end
fprintf(serial, 'f');
input = fread(serial, 1023, 'int8');
fclose(serial);
delete(serial);
clear serial;

plot(input);