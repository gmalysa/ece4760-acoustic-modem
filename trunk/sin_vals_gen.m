% Data parameters
t = [0:63];
fs = 40000;
f = [4800 6000 7200 10800 10000 11000 13000 15000];
weights = [.9 .5 .6 .8 .8 .9  1 1 ]';
fn = f ./ fs;

% Matrix of cosine arguments/values
n = fn'*t;
samples = (weights * hann(64)') .* cos(2*pi*n);

% Brute force coefficients
c = zeros(256, 8);
for i = 0:255
    a = 1;
    for j = 0:7
        if (bitand(a, i) == a)
            c(i+1, j+1) = 1;
        end
        a = a*2;
    end
end

% Now create a huge table of possible waveforms
data = zeros(256, 64);
for i = 0:255
    % Initial import
    data(i+1, :) = c(i+1, :) * samples;
    % Rescale
    if (max(abs(data(i+1,:))) > 1)
        data(i+1, :) = data(i+1, :) * 128/max(abs(data(i+1,:)));
    else
        data(i+1, :) = data(i+1, :) * 128;
    end
    % Adjust position and floor
    data(i+1, :) = floor(data(i+1, :) + 128);
end

data(data == 256) = 255;

csvwrite('sin_vals.c', data, 5, 0);