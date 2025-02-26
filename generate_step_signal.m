function  y = generate_step_signal(Fs, t_duration, step_time, level)
        t = 0:1/Fs:t_duration;
        y = zeros(size(t));
        y(t >= step_time) = level;
end