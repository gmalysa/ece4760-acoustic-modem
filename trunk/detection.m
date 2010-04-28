% Figure out how to detect and decode data off of the input lines

%clear;
clf;
figure(1);
hold on;

fs = 40000;
f = 9000;
buflen = ceil(64. / (fs/f));
thresh = 2000;

% Sample input, with some amount of offset from the start of our buffer
% data = hann(64)' .* 40 .*cos(2*pi*(f/fs)*[0:63]);
% input = zeros(1, 1024);
% input(offset:offset+63) = data;
% input(offset+64:offset+63+64) = data;
% input(offset+64+64+64:offset+63+64+64+64) = data;

try
    serial = serial('COM5', 'BaudRate', 230400, 'InputBufferSize', 1024, 'Terminator', '');
catch exception
    delete(serial);
    clear serial;
    serial = serial('COM5', 'BaudRate', 230400, 'InputBufferSize', 1024, 'Terminator', '');
end
try
    fopen(serial);
catch exception
    delete(serial);
    clear serial;
    serial = serial('COM5', 'BaudRate', 230400, 'InputBufferSize', 1024, 'Terminator', '');
    fopen(serial);
end
fprintf(serial, 'f');
input = fread(serial, 1023, 'int8');
fclose(serial);
delete(serial);
clear serial;

%input = x;

% Add noise for fun
%input = input + 4*randn(1, 1024);

% Run input through resampler
j = 1 + 8.*rand(1,1);
k = j + fs/(4*f);
step = fs/f;
i = 1;
l = 1;
rc = zeros(1, buflen);
rs = zeros(1, buflen);
ic = zeros(1, 1);
is = zeros(1, 1);
csum = 0;
ssum = 0;
start = 0;
samples = 0;
detected = 0;
prev_csum = 0;
prev_ssum = 0;
prev_derv_csum = 0;
prev_derv_ssum = 0;
while (k < length(input))
    % Find integer offsets
    cIndex = floor(j);
    sIndex = floor(k);
    
    % Update running structures
    prev_derv_csum = prev_csum - csum;
    prev_csum = csum;
    csum = csum - rc(i);
    rc(i) = input(cIndex);
    csum = csum + rc(i);
    prev_derv_ssum = prev_ssum - ssum;
    prev_ssum = ssum;
    ssum = ssum - rs(i);
    rs(i) = input(sIndex);
    ssum = ssum + rs(i);
    subplot(2, 1, 1);
    hold on;
    plot(j, csum, 'bx');
    plot(k, ssum, 'rx');
    subplot(2, 1, 2);
    hold on;
    plot(j, csum^2 + ssum^2, 'kx');
    ic(l) = j;
    is(l) = k;
    
    % Attempt detection
    trigger = 0;
    if (csum^2 + ssum^2) > thresh
        if abs(csum) > abs(ssum)
            subplot(2, 1, 1);
            plot(j, -140, 'bx');
            if ((prev_csum - csum) >= 0 && prev_derv_csum < 0) || ((prev_csum - csum) < 0 && prev_derv_csum >= 0)
                plot(j, -150, 'kx');
                trigger = 1;
            end
        else
            subplot(2, 1, 1);
            plot(j, 120, 'rx');
            if ((prev_ssum - ssum) >= 0 && prev_derv_ssum < 0) || ((prev_ssum - ssum) < 0 && prev_derv_ssum >= 0)
                plot(j, 130, 'kx');
                trigger = 1;
            end
        end
        if (0 == start && trigger == 1)
%             rc = zeros(1, buflen);
%             rs = zeros(1, buflen);
%             csum = 0;
%             ssum = 0;
            samples = buflen;
            start = 9;
%            j = j - 10;
%            k = k - 10;
        end
    end
    
    if (samples == buflen)
        subplot(2, 1, 1);
        line([cIndex cIndex], [-400 400]);
        samples = 0;
        if (start > 0)
            if (csum^2 + ssum^2) > thresh
                subplot(2, 1, 1);
                line([sIndex sIndex], [-200 200]);
                detected = 1;
            end
            rc = zeros(1, buflen);
            rs = zeros(1, buflen);
            csum = 0;
            ssum = 0;
            j = j - (step * buflen - 64);
            k = k - (step * buflen - 64);
        end
        
        if (detected && start > 0)
            start = start - 1;
        elseif (start > 0)
            start = start -1;
        end
        detected = 0;
    end
    
    % Update sample count (redundant)
    samples = samples + 1;
    
    % Update buffers/pointers/offsets
    j = j + step;
    k = k + step;
    i = i + 1;
    l = l + 1;
    if (i > buflen)
        i = 1;
    end
end

%plot(ic, rc, 'g');
subplot(2, 1, 1);
plot(input, 'b');
subplot(2, 1, 2);
plot([1:length(input)], 0, 'k');
%plot(is, rs, 'r');
