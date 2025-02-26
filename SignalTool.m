% MATLAB GUI 代码：信号采集与生成工具 
function varargout = SignalTool(varargin)
 
% 初始化 GUI 
gui_Singleton = true;
gui_State = struct('gui_Name', mfilename, ...
                   'gui_Singleton', gui_Singleton, ...
                   'gui_OpeningFcn', @SignalTool_OpeningFcn, ...
                   'gui_OutputFcn', @SignalTool_OutputFcn, ...
                   'gui_LayoutFcn', @UILayoutFcn, ...
                   'gui_Callback', []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback  = str2func(varargin{1});
end 
 
if nargout 
    [varargout{1:nargout}] = gui_OutsFcn(gui_State);
else 
    gui_Running = 1;
    gui_OpeningFcn(gui_State);
end 
 
% GUI 打开回调函数 
function SignalTool_OpeningFcn(hObject, eventdata, handles, varargin)
    handles.output  = hObject;
    guidata(hObject, handles);
    
    % 初始化信号类型下拉菜单 
    set(handles.SignalTypeDropDown, 'String', {'Sine', 'Square', 'Step', 'Chirp'});
    set(handles.SignalTypeDropDown ',Value', 1);
    
    % 初始化绘图区域 
    axes(handles.PlotAxes);
    xlabel('Time');
    ylabel('Amplitude');
    title('Signal Display');
end 
 
% GUI 输出回调函数 
function varargout = SignalTool_OutputFcn(hObject, eventdata, handles)
    varargout{1} = handles.output; 
end 
 
% UI 布局回调函数 
function UILayoutFcn(hObject, eventdata, handles)
    % 自动调整布局 
    set(hObject, 'Units', 'normalized');
    set(hObject, 'Position', [0.1 0.1 0.8 0.8]);
end 
 
% Start Recording 按钮回调函数 
function StartRecording_Callback(hObject, eventdata, handles)
    % 配置麦克风参数 
    Fs = 44100; % 采样率 
    Duration = 5; % 采集时间（秒）
    NumChannels = 2; % 两通道 
    
    % 启动录音 
    handles.Audio = audiorecorder(Fs, 'int16', NumChannels);
    record(handles.Audio);
    
    % 更新状态标签 
    set(handles.StatusLabel, 'String', 'Recording...');
    
    % 更新句柄 
    guidata(hObject, handles);
end 
 
% Stop Recording 按钮回调函数 
function StopRecording_Callback(hObject, eventdata, handles)
    if isvalid(handles.Audio)
        % 停止录音 
        stop(handles.Audio);
        audioData = getaudiodata(handles.Audio);
        
        % 清空绘图区域 
        cla(handles.PlotAxes);
        
        % 绘制采集的信号 
        t = linspace(0, length(audioData)/handles.Audio.Fs, length(audioData));
        plot(handles.PlotAxes, t, audioData(:,1), 'b'); % 第一通道 
        hold on;
        plot(handles.PlotAxes, t, audioData(:,2), 'r'); % 第二通道 
        legend('Channel 1', 'Channel 2');
        xlabel('Time (seconds)');
        ylabel('Amplitude');
        title('Recorded Audio Signal');
        
        % 更新状态标签 
        set(handles.StatusLabel, 'String', 'Recording stopped.');
    else 
        set(handles.StatusLabel, 'String', 'No recording in progress.');
    end 
end 
 
% Generate Signal 按钮回调函数 
function GenerateSignal_Callback(hObject, eventdata, handles)
    % 获取信号类型 
    signalType = get(handles.SignalTypeDropDown, 'Value');
    
    % 生成信号 
    Fs = 44100;
    t = linspace(0, 1, Fs); % 时间向量 
    
    switch(signalType)
        case 1 % 正弦波 
            signal = sin(2*pi*5*t);
            titleStr = 'Sine Wave';
        case 2 % 方波 
            signal = square(2*pi*5*t);
            titleStr = 'Square Wave';
        case 3 % 阶跃信号 
            signal = stepfun(t - 0.5);
            titleStr = 'Step Signal';
        case 4 % Chirp 信号 
            signal = chirp(t, 1, 1, 10);
            titleStr = 'Chirp Signal';
    end 
    
    % 清空绘图区域 
    cla(handles.PlotAxes);
    
    % 绘制信号 
    plot(handles.PlotAxes, t, signal);
    xlabel('Time (seconds)');
    ylabel('Amplitude');
    title(titleStr);
    
    % 更新状态标签 
    set(handles.StatusLabel, 'String', ['Generated ', titleStr]);
end 
 
% Close 按钮回调函数 
function Close_Callback(hObject, eventdata, handles)
    % 停止录音（如果正在录音）
    if isvalid(handles.Audio)
        stop(handles.Audio);
    end 
    
    % 关闭 GUI 
    delete(hObject);
end 