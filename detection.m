% Figure out how to detect and decode data off of the input lines

%clear;
clf;
figure(1);
hold on;

fs = 40000;
f = 4800;
f2 = 6000;
f3 = 7200;
buflen = ceil(64. / (fs/f));
thresh = 35000;
dt = -1000;

% Sample input, with some amount of offset from the start of our buffer
% data = hann(64)' .* 40 .*cos(2*pi*(f/fs)*[0:63]);
% input = zeros(1, 1024);
% input(offset:offset+63) = data;
% input(offset+64:offset+63+64) = data;
% input(offset+64+64+64:offset+63+64+64+64) = data;

COMs = 'COM9';
COMr = 'COM12';
bauds = 2400;
baudr = 230400;

try
    sserial = serial(COMs, 'BaudRate', bauds, 'InputBufferSize', 1024, 'Terminator', '', 'Timeout', 4);
catch exception
    delete(sserial);
    clear sserial;
    sserial = serial(COMs, 'BaudRate', bauds, 'InputBufferSize', 1024, 'Terminator', '', 'Timeout', 4);
end
try
    fopen(sserial);
catch exception
    delete(sserial);
    clear sserial;
    sserial = serial(COMs, 'BaudRate', bauds, 'InputBufferSize', 1024, 'Terminator', '', 'Timeout', 4);
    fopen(sserial);
end
try
    rserial = serial(COMr, 'BaudRate', baudr, 'InputBufferSize', 1024, 'Terminator', '', 'Timeout', 4);
catch exception
    delete(rserial);
    clear rserial;
    rserial = serial(COMr, 'BaudRate', baudr, 'InputBufferSize', 1024, 'Terminator', '', 'Timeout', 4);
end
try
    fopen(rserial);
catch exception
    delete(rserial);
    clear rserial;
    rserial = serial(COMr, 'BaudRate', baudr, 'InputBufferSize', 1024, 'Terminator', '', 'Timeout', 4);
    fopen(rserial);
end
fprintf(sserial, 'af');
%input = fread(rserial, 1023, 'int8');

fclose(rserial);
delete(rserial);
clear rserial;

fclose(sserial);
delete(sserial);
clear sserial;

%input = x;

% Add noise for fun
%input = input + 4*randn(1, 1024);

% Run input through resampler
j = 1;
k = j + fs/(4*f);
m = j + fs/(8*f);
n = j + (fs*3)/(8*f);
step = fs/f;
i = 1;
l = 1;
rc = zeros(1, buflen);
rs = zeros(1, buflen);
rm = zeros(1, buflen);
rn = zeros(1, buflen);
ic = zeros(1, 1);
is = zeros(1, 1);
im = zeros(1, 1);
in = zeros(1, 1);
mcs = zeros(1, 1);
mmn = zeros(1, 1);
csum = 0;
ssum = 0;
msum = 0;
nsum = 0;
start = 0;
samples = 0;
detected = 0;
prev_csum = 0;
prev_ssum = 0;
prev_derv_csum = 0;
prev_derv_ssum = 0;
prev_mag = 0;
prev_shift_mag = 0;
mag = 0;
while (n < length(input))
    % Find integer offsets
    cIndex = floor(j);
    sIndex = floor(k);
    mIndex = floor(m);
    nIndex = floor(n);
    
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
    msum = msum - rm(i);
    rm(i) = input(mIndex);
    msum = msum + rm(i);
    nsum = nsum - rn(i);
    rn(i) = input(nIndex);
    nsum = nsum + rn(i);
    
    subplot(2, 1, 1);
    hold on;
    plot(j, csum, 'bx');
    plot(k, ssum, 'rx');
    plot(m, msum, 'gx');
    plot(n, nsum, 'cx');
    subplot(2, 1, 2);
    hold on;
%    plot(j, csum^2 + ssum^2, 'kx');
%    plot(m, msum^2 + nsum^2, 'rx');
    ic(l) = j;
    is(l) = k;
    im(l) = m;
    in(l) = n;
    prev_mag = mcs(max(1,l-1));
    prev_shift_mag = mmn(max(1,l-1));
    mcs(l) = csum^2 + ssum^2;
    mmn(l) = msum^2 + nsum^2;
    
    % Attempt detection
    trigger = 0;
    mag = mean(csum^2 + ssum^2, msum^2 + nsum^2);
    if mag > thresh
%         if abs(csum) > abs(ssum)
%             subplot(2, 1, 1);
%             plot(j, -140, 'bx');
%             %if ((prev_csum - csum) >= 0 && prev_derv_csum < 0) || ((prev_csum - csum) < 0 && prev_derv_csum >= 0)
%             if (abs(abs(prev_csum - csum) - abs(prev_derv_csum)) < 10)
%                 plot(j, -150, 'kx');
%                 trigger = 1;
%             end
%         else
%             subplot(2, 1, 1);
%             plot(j, 120, 'rx');
%             %if ((prev_ssum - ssum) >= 0 && prev_derv_ssum < 0) || ((prev_ssum - ssum) < 0 && prev_derv_ssum >= 0)
%             if (abs(abs(prev_ssum - ssum) - abs(prev_derv_ssum)) < 10)
%                 plot(j, 130, 'kx');
%                 trigger = 1;
%             end
%         end
        if (mcs(l) > mmn(l))
            if (mag - prev_mag) < dt 
                trigger = 1;
            end
        else 
            if (mag - prev_shift_mag) < dt
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
            if mag > thresh
                subplot(2, 1, 1);
                line([sIndex sIndex], [-200 200]);
                detected = 1;
            end
%             rc = zeros(1, buflen);
%             rs = zeros(1, buflen);
%             rm = zeros(1, buflen);
%             rn = zeros(1, buflen);
%             csum = 0;5
%             ssum = 0;
%             msum = 0;
%             nsum = 0;
            j = j - (step * buflen - 64) + step/2;
            k = k - (step * buflen - 64) + step/2;
            m = m - (step * buflen - 64) + step/2;
            n = n - (step * buflen - 64) + step/2;
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
    %prev_mag = mag;
    
    % Update buffers/pointers/offsets
    j = j + step;
    k = k + step;
    m = m + step;
    n = n + step;
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
plot(im, mmn, 'r');
plot((ic + im) / 2, (mcs+ mmn)./2, 'g');
plot([1:length(input)], 0, 'k');
%plot(is, rs, 'r');
