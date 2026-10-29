function BUILD_QUAD_MODEL()
%BUILD_QUAD_MODEL  生成 models/quad_closed_loop.slx。
%   图表代码由 +uav/+model 生成，物理参数写成字面量，模型自身不依赖 MATLAB 路径。
%   与脚本实现对拍见 scripts/validate_model.m。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
outDir = fullfile(root, 'models');
if ~isfolder(outDir)
    mkdir(outDir);
end

p = uav.cfg.quad();
c = uav.cfg.ctrl();

mdl = 'quad_closed_loop';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);

% ---- 参考信号 ----
add_block('simulink/Sources/Step', [mdl '/StepPn'], 'Position', [30 40 60 70]);
set_param([mdl '/StepPn'], 'Time', '1', 'Before', '0', 'After', '3');

add_block('simulink/Sources/Step', [mdl '/StepPe'], 'Position', [30 100 60 130]);
set_param([mdl '/StepPe'], 'Time', '1', 'Before', '0', 'After', '2');

add_block('simulink/Sources/Constant', [mdl '/Pd'], 'Position', [30 160 60 190]);
set_param([mdl '/Pd'], 'Value', '-5');

add_block('simulink/Sources/Constant', [mdl '/Psi'], 'Position', [30 220 60 250]);
set_param([mdl '/Psi'], 'Value', '0.4');

add_block('simulink/Signal Routing/Mux', [mdl '/RefMux'], 'Position', [100 60 105 250]);
set_param([mdl '/RefMux'], 'Inputs', '4');

% ---- 计算块 ----
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Ctrl'], ...
    'Position', [150 40 260 240]);
set_chart(mdl, 'Ctrl', uav.model.ctrl_code(p, c), true);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Mixer'], ...
    'Position', [300 80 380 200]);
set_chart(mdl, 'Mixer', uav.model.mixer_code(p), false);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Motors'], ...
    'Position', [430 80 510 200]);
set_chart(mdl, 'Motors', uav.model.motors_code(p), true);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Wrench'], ...
    'Position', [560 40 670 240]);
set_chart(mdl, 'Wrench', uav.model.wrench_code(p), false);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Deriv'], ...
    'Position', [750 40 860 240]);
set_chart(mdl, 'Deriv', uav.model.deriv_code(p), false);

add_block('simulink/Signal Routing/Mux', [mdl '/TM'], 'Position', [280 270 285 320]);
set_param([mdl '/TM'], 'Inputs', '2');

add_block('simulink/Signal Routing/Mux', [mdl '/FM'], 'Position', [710 270 715 320]);
set_param([mdl '/FM'], 'Inputs', '2');

add_block('simulink/Continuous/Integrator', [mdl '/Int'], ...
    'Position', [900 130 940 170]);
set_param([mdl '/Int'], 'InitialCondition', '[0;0;-5;0;0;0;1;0;0;0;0;0;0]');

% ---- 记录 ----
add_block('simulink/Sinks/To Workspace', [mdl '/LogX'], 'Position', [980 120 1030 160]);
set_param([mdl '/LogX'], 'VariableName', 'x_out', 'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [mdl '/LogW'], 'Position', [560 320 610 360]);
set_param([mdl '/LogW'], 'VariableName', 'w_out', 'SaveFormat', 'Timeseries');

% ---- 连线 ----
add_line(mdl, 'StepPn/1', 'RefMux/1');
add_line(mdl, 'StepPe/1', 'RefMux/2');
add_line(mdl, 'Pd/1', 'RefMux/3');
add_line(mdl, 'Psi/1', 'RefMux/4');

add_line(mdl, 'Int/1', 'Ctrl/1');
add_line(mdl, 'RefMux/1', 'Ctrl/2');
add_line(mdl, 'Ctrl/1', 'TM/1');
add_line(mdl, 'Ctrl/2', 'TM/2');
add_line(mdl, 'TM/1', 'Mixer/1');
add_line(mdl, 'Mixer/1', 'Motors/1');
add_line(mdl, 'Motors/1', 'Wrench/1');
add_line(mdl, 'Int/1', 'Wrench/2');
add_line(mdl, 'Wrench/1', 'FM/1');
add_line(mdl, 'Wrench/2', 'FM/2');
add_line(mdl, 'Int/1', 'Deriv/1');
add_line(mdl, 'FM/1', 'Deriv/2');
add_line(mdl, 'Deriv/1', 'Int/1');
add_line(mdl, 'Int/1', 'LogX/1');
add_line(mdl, 'Motors/1', 'LogW/1');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', '1e-3', 'StopTime', '14', ...
    'SolverType', 'Fixed-step');

save_system(mdl, fullfile(outDir, [mdl '.slx']));
fprintf('已生成 %s\n', fullfile(outDir, [mdl '.slx']));
end

% ------------------------------------------------------------------
function set_chart(mdl, name, code, discrete)
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [mdl '/' name]);
if isempty(ch)
    error('build:chartNotFound', '未找到 %s 的图表对象', name);
end
ch.Script = code;
if discrete
    % 带持久变量的图表不能继承连续采样时间
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime  = '1e-3';
end
end
