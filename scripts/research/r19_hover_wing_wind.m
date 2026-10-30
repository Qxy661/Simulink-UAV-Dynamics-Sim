function T = r19_hover_wing_wind()
%R19_HOVER_WING_WIND  悬停抗风上限到底是控制器还是机翼决定的。
%   固定控制器与 ESO，只改机翼面积，看 6/8 m/s 侧风能不能守住。
%   结论见 research/REPORT_r15_hover_wing_wind.md。
%
%   跑完会打印三组：S=1.6（原机）、0.8、0.2。

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root); cd(root);

p0 = uav.cfg.tilt();
c  = uav.cfg.tiltctrl();
s0 = uav.cfg.simcfg();
s0.tEnd = 20; s0.logDiv = 10;

ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                  'psi', 0, 'h', 20, 'V', 0);

Slist = [1.6 0.8 0.2];
Wlist = [6 8];
T = table();

for S = Slist
    p = p0; p.S = S;
    for W = Wlist
        s = s0;
        s.windFn = @(tt) [W; 0; 0];
        s.useEso = true; s.woEso = 8;
        o = uav.sim.run_tilt(p, c, s, ref);
        m = o.t > o.t(end) - 10;
        e = rad2deg(o.eul);
        errTail = max(vecnorm(o.p(m,1:2), 2, 2));
        tiltMax = max(max(abs(e(m,1:2))));
        T = [T; table(S, W, errTail, tiltMax, ...
            'VariableNames', {'S','wind','errTail','tiltMax'})]; %#ok<AGROW>
        fprintf('S=%.1f W=%2.0f  err=%7.2f  tilt=%6.1f\n', S, W, errTail, tiltMax);
    end
end

csvDir = fullfile(root, 'research', 'results', 'csv');
if ~isfolder(csvDir), mkdir(csvDir); end
writetable(T, fullfile(csvDir, 'r19_hover_wing_wind.csv'));
end
