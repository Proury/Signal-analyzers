

function y = generate_square_wave(Fs, t_duration, f)
    t = 0:1/Fs:t_duration;
    y = square(2*pi*f*t);
end

