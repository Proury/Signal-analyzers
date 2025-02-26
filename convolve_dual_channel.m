function [conv_left_right,conv_right_left] = convolve_dual_channel(left_channel,right_channel)
        % 左右两声道进行卷积
        % 卷积属于线性操作，conv(left,right) = conv(right,left)
        conv_left_right  = conv(left_channel,right_channel);

        conv_right_left  = conv(right_channel,left_channel);
end
