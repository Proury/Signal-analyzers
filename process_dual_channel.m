function [sum_signal, diff_signal, product_signal] = process_dual_channel(left_channel, right_channel)
    % 相加操作
    sum_signal = left_channel + right_channel;
    
    % 相减操作
    diff_signal = left_channel - right_channel;
    
    % 相乘操作
    product_signal = left_channel .* right_channel;
end
