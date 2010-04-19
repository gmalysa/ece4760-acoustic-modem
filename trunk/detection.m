% Figure out how to detect and decode data off of the input lines

clear;
clf;
figure(1);
hold on;

fs = 40000;
f = 4000;
buflen = 7;
offset = 27;

% Sample input, with some amount of offset from the start of our buffer
data = hann(64)' .* cos(2*pi*(f/fs)*[0:63]) * 128;
input = zeros(1, 1024);
input(offset:offset+63) = data;
input(offset+64:offset+63+64) = data;
input(offset+64+64+64:offset+63+64+64+64) = data;

% Add noise for fun
input = input + 8*randn(1, 1024);

% Run input through resampler
j = 1;
k = 1 + fs/(4*f);
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
while (k < length(input))
    % Find integer offsets
    cIndex = floor(j);
    sIndex = floor(k);
    
    % Update running structures
    csum = csum - rc(i);
    rc(i) = input(cIndex);
    csum = csum + rc(i);
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
    if (csum^2 + ssum^2) > 70000
        detected = 1;
        rc = zeros(1, buflen);
        rs = zeros(1, buflen);
        csum = 0;
        ssum = 0;
        if (0 == start)
            samples = buflen;
            start = 9;
        end
    end
    
    if (samples == buflen)
        samples = 0;
        j = j + 4;
        k = k + 4;
        if (detected && start > 0)
            display('1');
            start = start - 1;
        elseif (start > 0)
            display('0');
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
plot([1:512], 0, 'k');
%plot(is, rs, 'r');
