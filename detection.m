% Figure out how to detect and decode data off of the input lines

%clear;
clf;
figure(1);
hold on;

fs = 40000;
f = 4000;
buflen = 7;
offset = 27;
thresh = 6000;

% Sample input, with some amount of offset from the start of our buffer
data = hann(64)' .* 40 .*cos(2*pi*(f/fs)*[0:63]);
input = zeros(1, 1024);
input(offset:offset+63) = data;
input(offset+64:offset+63+64) = data;
input(offset+64+64+64:offset+63+64+64+64) = data;

input = x;

% Add noise for fun
%input = input + 4*randn(1, 1024);

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
    if (csum^2 + ssum^2) > thresh
        if (0 == start)
%             rc = zeros(1, buflen);
%             rs = zeros(1, buflen);
%             csum = 0;
%             ssum = 0;
            samples = buflen;
            start = 9;
            j = j + 10;
            k = k + 10;
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
            j = j - 6;
            k = k - 6;
        end
        
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
plot([1:length(input)], 0, 'k');
%plot(is, rs, 'r');
