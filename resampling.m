% Simulator for the acoustic modem doing all of its calculations by
% resampling the single input signal

clear;
clf;

% Parameters
fs = 40000;
f = 10000;
fguess = 9000;
phi = pi/2;
fdelta = 7000;
fstep = 50;

% 64 samples at 40 kHz is 1.6 ms of data
t = [0:63];
noise = 0.25*randn(1, length(t));

figure(1);
hold on;

% Input frequencies
f = [4000, 6000, 7000, 9000, 10000, 11000, 13000, 15000];
% Experimentally the best resampling frequency for each is:
fre = [4050, 6100, 7000, 9000, 10000, 11000, 13000, 15000];
color = ['b' 'g' 'r' 'c' 'k' 'y', 'm', 'x'];
for a = 1:length(f)
    for phi = 0:0.1:pi/2
        data = hann(64)' .* cos(2*pi*t*f(a)/fs + phi);% + noise;
        i = 1;
        for n = -fdelta:fstep:fdelta
            rs = zeros(1, 1);
            rc = zeros(1, 1);
            k = 1;
            j = 1;
            l = 1 + fs/(4*(fguess+n));
            while (j < length(data)-1) && (l < length(data)-1)
                intIndex = floor(j);
                intIndex2 = floor(l);
                rs(k) = data(intIndex);
                rc(k) = data(intIndex2);
                k = k + 1;
                j = j + fs/(fguess+n);
                l = l + fs/(fguess+n);
            end
            result(i) = sum(rs)^2 + sum(rc)^2;
            i = i+1;
        end
        figure(1);
        plot(fguess+[-fdelta:fstep:fdelta], result, color(a));
    end
end