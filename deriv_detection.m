% Figure out how to detect and decode data off of the input lines

%clear;
clf;
figure(1);
hold on;

% Frequencies all fit in each other's zeros!
% Therefore, use: 4800, 6000, 8400, 10800 for the first four frequencies of
% interest.

% Parameters
fs = 40000;
f = 6000;
buflen = ceil(64. / (fs/f));
thresh = 140;
dt = 50;
offset = 69;

% Edge definitions
NO_EDGE = 0;
RISING_EDGE = 1;        % Going up
PEAK = 3;               % Make sure we level off
FALLING_EDGE = 4;       % Completed falling edge

% Sample input, with some amount of offset from the start of our buffer
data = hann(64)' .* 40 .*cos(2*pi*(10800/fs)*[0:63]);
data2 = hann(64)' .* 40 .*cos(2*pi*(8400/fs)*[0:63]);
data3 = hann(64)' .* 40 .*cos(2*pi*(6000/fs)*[0:63]);
data4 = hann(64)' .* 40 .*cos(2*pi*(4800/fs)*[0:63]);
% input = zeros(1, 1024);
% input(offset:offset+63) = data + data2;
% input(offset+64:offset+63+64) = data + data2;
% input(offset+64+64:offset+64+64+63) = data4;
% input(offset+64+64+64:offset+63+64+64+64) = data2+data3;

COMs = 'COM9';
COMr = 'COM12';
bauds = 2400;
baudr = 230400;

try
    sserial = serial(COMs, 'BaudRate', bauds, 'InputBufferSize', 1024, 'Terminator', '');
catch exception
    delete(sserial);
    clear sserial;
    sserial = serial(COMs, 'BaudRate', bauds, 'InputBufferSize', 1024, 'Terminator', '');
end
try
    fopen(sserial);
catch exception
    delete(sserial);
    clear sserial;
    sserial = serial(COMs, 'BaudRate', bauds, 'InputBufferSize', 1024, 'Terminator', '');
    fopen(sserial);
end
try
    rserial = serial(COMr, 'BaudRate', baudr, 'InputBufferSize', 1024, 'Terminator', '');
catch exception
    delete(rserial);
    clear rserial;
    rserial = serial(COMr, 'BaudRate', baudr, 'InputBufferSize', 1024, 'Terminator', '');
end
try
    fopen(rserial);
catch exception
    delete(rserial);
    clear rserial;
    rserial = serial(COMr, 'BaudRate', baudr, 'InputBufferSize', 1024, 'Terminator', '');
    fopen(rserial);
end
fprintf(sserial, 'hs');
input = fread(rserial, 1023, 'int8');
fclose(rserial);
delete(rserial);
fclose(sserial);
delete(sserial);
clear sserial;
clear rserial;

%input = x;

% Add noise for fun
%input = input + 4*randn(1, 1024);

% Run input through resampler
j = 11;
k = j + fs/(4*f);
% m = j + fs/(8*f);
% n = j + (3*fs)/(8*f);
step = fs/f;
i = 1;
l = 1;
rc = zeros(1, buflen);
rs = zeros(1, buflen);
% rm = zeros(1, buflen);
% rn = zeros(1, buflen);
ic = zeros(1, 1);
is = zeros(1, 1);
% im = zeros(1, 1);
% in = zeros(1, 1);
mcs = zeros(1, 1);
% mmn = zeros(1, 1);
csum = 0;
ssum = 0;
% msum = 0;
% nsum = 0;
start = 0;
samples = 0;
detected = 0;
prev_mag = 0;
% prev_shift_mag = 0;
mag = 0;
trigger = 0;    % No edge
while (k < length(input))
    % Find integer offsets
    cIndex = floor(j);
    sIndex = floor(k);
%     mIndex = floor(m);
%     nIndex = floor(n);
    
    % Update running structures
    csum = csum - rc(i);
    rc(i) = input(cIndex);
    csum = csum + rc(i);
    ssum = ssum - rs(i);
    rs(i) = input(sIndex);
    ssum = ssum + rs(i);
%     msum = msum - rm(i);
%     rm(i) = input(mIndex);
%     msum = msum + rm(i);
%     nsum = nsum - rn(i);
%     rn(i) = input(nIndex);
%     nsum = nsum + rn(i);

    subplot(2, 1, 1);
    hold on;
    plot(j, csum, 'bx');
    plot(k, ssum, 'rx');
    subplot(2, 1, 2);
    hold on;
    ic(l) = j;
    is(l) = k;
    prev_mag = mcs(max(1,l-1));
%     prev_shift_mag = mmn(max(1, l-1));
    
    % Calculate our signal magnitude
%     mag = max(abs(csum), abs(ssum));
    mag = max(abs(csum), abs(ssum));
%     shift_mag = max(abs(msum), abs(nsum));
    mcs(l) = mag;
%     mmn(l) = shift_mag;
    
    subplot(2, 1, 2);
    switch(trigger)
        case NO_EDGE
            plot(j, 40, 'x');
        case RISING_EDGE
            plot(j, 60, 'x');
        case PEAK
            plot(j, 80, 'x');
    end

    % Determine which edge we are on should we cross the threshold
    if (start == 0)
        switch (trigger)
            case NO_EDGE
                if (mag > thresh)% &&  mag > prev_mag%prev_mag - mag < 20
                    trigger = RISING_EDGE;
                end
            case RISING_EDGE
                % Detect a leveling off
                if (abs(mag - prev_mag) < dt) && mag > thresh
                    trigger = PEAK;
                % Fall back if we don't level off
                else
                    trigger = NO_EDGE;
                    plot(j, 20, 'rx');
                end
            case PEAK
                if prev_mag - mag > -dt
                    trigger = FALLING_EDGE;
                elseif (mag < thresh)
                    trigger = NO_EDGE;
                    plot(j, 20, 'gx');
                end
        end
    end
    
        % Note the start of the frame
        if (0 == start && trigger == FALLING_EDGE)
            start = 9;
            trigger = NO_EDGE;
            detected = 1;
            samples = buflen;
%         elseif (trigger == FALLING_EDGE)
%             % Else note that for this frame we are definitely
%             % detecting a 1 bit
%             detected = 1;
%             trigger = NO_EDGE;
        end
    
     % Check for end of frame
    if (samples == buflen)
        subplot(2, 1, 1);
        line([cIndex cIndex], [-400 400]);
        samples = 0;
        if (start > 0)
            % Draw the detected line if appropriate
            if (detected || mag > thresh)
                trigger = NO_EDGE;
                subplot(2, 1, 1);
                line([sIndex sIndex], [-200 200]);
            end
        end
        
        % Advance input parser state machine
        if (start > 0)
            start = start - 1;
            j = j - (step * buflen - 64);
            k = k - (step * buflen - 64);
            if (start == 0)
                rs = zeros(1, buflen);
                rc = zeros(1, buflen);
                csum = 0;
                ssum = 0;
                
            end
        end
        detected = 0;
    end
    
    % Update sample count
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
line([323, 323], [-600 600]);
subplot(2, 1, 2);
hold on;
plot(ic, mcs, 'k');
% plot(im, mmn, 'r');
plot([1:length(input)], 0, 'k');
%plot(is, rs, 'r');
