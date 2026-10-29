function BUILD_TILT_MODEL()
%BUILD_TILT_MODEL  生成 models/tilt_closed_loop.slx。
%   三个图表：参考（纯函数）、控制器（离散 1e-3）、21 维导数（纯函数）。
%   参数写成字面量，模型不依赖 MATLAB 路径；与脚本实现对拍见 validate_tilt_model.m。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
outDir = fullfile(root, 'models');
if ~isfolder(outDir)
    mkdir(outDir);
end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();

mdl = 'tilt_closed_loop';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);

% ---- 时间与参考 ----
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [30 40 60 70]);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Ref'], ...
    'Position', [100 30 240 90]);
set_chart(mdl, 'Ref', uav.model.tilt_ref_code(), false);

% ---- 控制器与导数 ----
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Ctrl'], ...
    'Position', [280 40 440 200]);
set_chart(mdl, 'Ctrl', uav.model.tilt_ctrl_code(p, c), true);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Deriv'], ...
    'Position', [480 40 660 200]);
set_chart(mdl, 'Deriv', uav.model.tilt_deriv_code(p), false);

add_block('simulink/Continuous/Integrator', [mdl '/Int'], ...
    'Position', [700 110 740 150]);
set_param([mdl '/Int'], 'InitialCondition', mat2str(tilt_ic(p), 17));

% ---- 记录 ----
add_block('simulink/Sinks/To Workspace', [mdl '/LogX'], 'Position', [790 110 840 150]);
set_param([mdl '/LogX'], 'VariableName', 'x_out', 'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [mdl '/LogCmd'], 'Position', [480 250 530 290]);
set_param([mdl '/LogCmd'], 'VariableName', 'cmd_out', 'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [mdl '/LogRef'], 'Position', [200 250 250 290]);
set_param([mdl '/LogRef'], 'VariableName', 'ref_out', 'SaveFormat', 'Timeseries');

% ---- 连线 ----
add_line(mdl, 'Clock/1', 'Ref/1');
add_line(mdl, 'Int/1', 'Ctrl/1');
add_line(mdl, 'Ref/1', 'Ctrl/2');
add_line(mdl, 'Int/1', 'Deriv/1');
add_line(mdl, 'Ctrl/1', 'Deriv/2');
add_line(mdl, 'Deriv/1', 'Int/1');
add_line(mdl, 'Int/1', 'LogX/1');
add_line(mdl, 'Ctrl/1', 'LogCmd/1');
add_line(mdl, 'Ref/1', 'LogRef/1');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', '1e-3', 'StopTime', '45', ...
    'SolverType', 'Fixed-step');

save_system(mdl, fullfile(outDir, [mdl '.slx']));
fprintf('已生成 %s\n', fullfile(outDir, [mdl '.slx']));
end

% ------------------------------------------------------------------
function x0 = tilt_ic(p)
q = [1; 0; 0; 0];
x0 = [0; 0; -p.htrim; 0; 0; 0; q; 0; 0; 0; p.betaMin; p.hoverW*ones(4,1); 0; 0; 0];
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
