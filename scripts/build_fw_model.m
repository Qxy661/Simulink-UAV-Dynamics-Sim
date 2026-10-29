function BUILD_FW_MODEL()
%BUILD_FW_MODEL  生成 models/fw_closed_loop.slx。
%   三个图表：控制器（离散 1e-3）、执行机构（离散 1e-3）、刚体导数（纯函数）。
%   参数写成字面量，模型不依赖 MATLAB 路径；与脚本实现对拍见 validate_fw_model.m。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
outDir = fullfile(root, 'models');
if ~isfolder(outDir)
    mkdir(outDir);
end

p  = uav.cfg.fw();
c  = uav.cfg.fwctrl();
tr = uav.plant.fw_trim(p, p.Vtrim);

mdl = 'fw_closed_loop';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);

% ---- 参考信号：高度、航向、空速 ----
add_block('simulink/Sources/Step', [mdl '/StepH'], 'Position', [30 40 60 70]);
set_param([mdl '/StepH'], 'Time', '5', 'Before', '100', 'After', '130');

add_block('simulink/Sources/Step', [mdl '/StepPsi'], 'Position', [30 110 60 140]);
set_param([mdl '/StepPsi'], 'Time', '20', 'Before', '0', 'After', num2str(60*pi/180, '%.17g'));

add_block('simulink/Sources/Step', [mdl '/StepV'], 'Position', [30 180 60 210]);
set_param([mdl '/StepV'], 'Time', '35', 'Before', '18', 'After', '22');

add_block('simulink/Signal Routing/Mux', [mdl '/RefMux'], 'Position', [100 50 105 210]);
set_param([mdl '/RefMux'], 'Inputs', '3');

% ---- 计算块 ----
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Ctrl'], ...
    'Position', [160 40 300 220]);
set_chart(mdl, 'Ctrl', uav.model.fw_ctrl_code(p, c), true);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Act'], ...
    'Position', [350 80 450 180]);
set_chart(mdl, 'Act', uav.model.fw_act_code(p, tr), true);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Plant'], ...
    'Position', [500 40 640 220]);
set_chart(mdl, 'Plant', uav.model.fw_plant_code(p), false);

add_block('simulink/Continuous/Integrator', [mdl '/Int'], ...
    'Position', [690 120 730 160]);
set_param([mdl '/Int'], 'InitialCondition', ...
    mat2str(fw_ic(p, tr), 17));

% ---- 记录 ----
add_block('simulink/Sinks/To Workspace', [mdl '/LogX'], 'Position', [780 110 830 150]);
set_param([mdl '/LogX'], 'VariableName', 'x_out', 'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [mdl '/LogAct'], 'Position', [500 280 550 320]);
set_param([mdl '/LogAct'], 'VariableName', 'act_out', 'SaveFormat', 'Timeseries');

% ---- 连线 ----
add_line(mdl, 'StepH/1', 'RefMux/1');
add_line(mdl, 'StepPsi/1', 'RefMux/2');
add_line(mdl, 'StepV/1', 'RefMux/3');

add_line(mdl, 'Int/1', 'Ctrl/1');
add_line(mdl, 'RefMux/1', 'Ctrl/2');
add_line(mdl, 'Ctrl/1', 'Act/1');
add_line(mdl, 'Int/1', 'Plant/1');
add_line(mdl, 'Act/1', 'Plant/2');
add_line(mdl, 'Plant/1', 'Int/1');
add_line(mdl, 'Int/1', 'LogX/1');
add_line(mdl, 'Act/1', 'LogAct/1');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', '1e-3', 'StopTime', '60', ...
    'SolverType', 'Fixed-step');

save_system(mdl, fullfile(outDir, [mdl '.slx']));
fprintf('已生成 %s\n', fullfile(outDir, [mdl '.slx']));
end

% ------------------------------------------------------------------
function x0 = fw_ic(p, tr)
q  = uav.plant.euler2quat(0, tr.alpha, 0);
Vb = p.Vtrim * [cos(tr.alpha); 0; sin(tr.alpha)];
x0 = [0; 0; -p.htrim; Vb; q; 0; 0; 0];
end

function set_chart(mdl, name, code, discrete)
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [mdl '/' name]);
if isempty(ch)
    error('build:chartNotFound', '未找到 %s 的图表对象', name);
end
ch.Script = code;
if discrete
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime  = '1e-3';
end
end
