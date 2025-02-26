function y = generate_sine_wave(Fs, t_duration, f , A , phi)
    t = 0:1/Fs:t_duration;
    y = A*sin(2*pi*f*t + phi);
end
