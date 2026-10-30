function r7_feasibility(Vlist, Tlist)
%R7_FEASIBILITY 减速段的可行性图谱：巡航速度 × 减速时长 -> 高度代价。
%
%   r7_feasibility               默认 V=[16 19 22 25]，Td=[12 15 20 25]
%   r7_feasibility(Vlist, Tlist)
%
%   每个格点跑一次 80 s 往返任务（20 m 巡航、掉到 8 m 悬停），记录：
%     dhPeak   全程高度偏差峰值 (m)      —— 减速段的高度代价
%     vEnd     末端空速误差 (m/s)        —— 是否真的停下来
%     aPeak    实际减速度峰值 (m/s^2)    —— 对比参考峰值，看跟踪能力
%     thMax    俯仰峰值 (deg)
%     satMax   推力占比峰值              —— 是否靠饱和硬顶
%
%   输出 research/results/csv/r7_feasibility.csv 与同名热图。

if nargin < 1 || isempty(Vlist), Vlist = [16 19 22 25]; end
if nargin < 2 || isempty(Tlist), Tlist = [12 15 20 25]; end

p = uav.cfg.tilt();
c = uav.cfg.tiltctrl();
s = uav.cfg.simcfg();
s.tEnd = 80;
s.logDiv = 10;
s.ctrlDiv = 10;

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
csvDir = fullfile(root, 'research', 'results', 'csv');
figDir = fullfile(root, 'research', 'results', 'fig');
if ~isfolder(csvDir), mkdir(csvDir); end
if ~isfolder(figDir), mkdir(figDir); end

nV = numel(Vlist); nT = numel(Tlist);
dhPeak = nan(nV, nT); vEnd = nan(nV, nT); aPeak = nan(nV, nT);
thMax  = nan(nV, nT); satMax = nan(nV, nT); aRef = nan(nV, nT);

fprintf('\n%6s %6s %9s %8s %8s %8s %8s\n', 'Vmax', 'Tdec', 'dhPeak', 'vEnd', 'aAct', 'aRef', 'sat');
for i = 1:nV
    for j = 1:nT
        Vmax = Vlist(i); Td = Tlist(j);
        ref = @(t) uav.env.tilt_ref_bt(t, Vmax, 20, 8, Td);
        try
            out = uav.sim.run_tilt(p, c, s, ref);
        catch ME
            fprintf('%6.0f %6.0f  FAILED: %s\n', Vmax, Td, ME.message);
            continue
        end
        t  = out.t;
        V  = vecnorm(out.v, 2, 2);
        h  = -out.p(:, 3);
        dh = h - out.ref(:, 5);
        aA = gradient(V, mean(diff(t)));
        aR = gradient(out.ref(:, 6), mean(diff(t)));
        U1 = p.kt*sum(out.om.^2, 2);
        dhPeak(i,j) = max(abs(dh));
        vEnd(i,j)   = abs(V(end));
        aPeak(i,j)  = max(abs(aA));
        aRef(i,j)   = max(abs(aR));
        thMax(i,j)  = max(abs(rad2deg(out.eul(:, 2))));
        satMax(i,j) = max(U1)/(4*p.kt*p.wMax^2);
        fprintf('%6.0f %6.0f %9.2f %8.2f %8.2f %8.2f %8.2f\n', ...
            Vmax, Td, dhPeak(i,j), vEnd(i,j), aPeak(i,j), aRef(i,j), satMax(i,j));
    end
end

T = table(repmat(Vlist(:), nT, 1), ...
          reshape(repmat(Tlist(:)', nV, 1), [], 1), ...
          reshape(dhPeak, [], 1), reshape(vEnd, [], 1), ...
          reshape(aPeak, [], 1), reshape(aRef, [], 1), ...
          reshape(thMax, [], 1), reshape(satMax, [], 1), ...
          'VariableNames', {'Vmax_mps','Tdec_s','dhPeak_m','vEnd_mps','aAct_mps2','aRef_mps2','thMax_deg','satFrac'});
writetable(T, fullfile(csvDir, 'r7_feasibility.csv'));

f = figure('Color','w','Position',[80 80 900 420]);
subplot(1,2,1);
imagesc(Tlist, Vlist, dhPeak); colorbar; axis xy;
xlabel('减速时长 T_{dec} (s)'); ylabel('巡航速度 (m/s)'); title('高度代价峰值 (m)');
subplot(1,2,2);
imagesc(Tlist, Vlist, vEnd); colorbar; axis xy;
xlabel('减速时长 T_{dec} (s)'); ylabel('巡航速度 (m/s)'); title('末端空速 (m/s)');
exportgraphics(f, fullfile(figDir, 'r7_feasibility.png'));
fprintf('\n写入 research/results/csv/r7_feasibility.csv\n');
end
