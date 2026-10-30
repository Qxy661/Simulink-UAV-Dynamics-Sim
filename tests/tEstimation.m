classdef tEstimation < matlab.unittest.TestCase
%TESTIMATION 传感器模型与 EKF 的单元测试。
%   运行：runtests('tests/tEstimation.m')

    properties
        p
        c
    end

    methods (TestClassSetup)
        function setupOnce(tc)
            tc.p = uav.cfg.quad();
            tc.c = uav.cfg.ctrl();
        end
    end

    methods (Test)
        function gyroBiasConverges(tc)
            % 零偏 0.02 rad/s，60 s 内应估到 10% 以内
            s = tc.fastCfg(60);
            out = uav.sim.run_est(tc.p, tc.c, s);
            bgErr = abs(out.bg(end, :) - s.sens.bg');
            tc.verifyLessThan(max(bgErr), 0.1 * max(abs(s.sens.bg)));
        end

        function gpsDropoutDriftsSlowly(tc)
            % 30 s 丢星，靠光流维持，水平漂移 < 5 m
            s = tc.fastCfg(60);
            s.sens.gpsDrop = [15, 45];
            out = uav.sim.run_est(tc.p, tc.c, s);
            drift = norm(out.p(end, 1:2) - out.ref(end, 1:2));
            tc.verifyLessThan(drift, 5);
        end

        function idealSensorsTrackHover(tc)
            % 无噪声闭环：末端 < 10 cm
            s = tc.fastCfg(30);
            s.sens = struct('sigmaG', 1e-6, 'sigmaA', 1e-6, 'sigmaGps', [1e-6; 1e-6; 1e-6], ...
                'sigmaGpsVel', 1e-6, 'sigmaBaro', 1e-6, 'sigmaMag', 1e-6, ...
                'bg', [0; 0; 0], 'ba', [0; 0; 0], 'useFlow', false);
            out = uav.sim.run_est(tc.p, tc.c, s);
            tc.verifyLessThan(norm(out.p(end, :) - out.ref(end, :)), 0.10);
        end

        function flowAidingImprovesHover(tc)
            % 同样噪声下，光流辅助应把 30 s 后的 RMS 误差压到一半以下
            s1 = tc.fastCfg(45);
            s1.sens.useFlow = false;
            s2 = s1;
            s2.sens.useFlow = true;
            o1 = uav.sim.run_est(tc.p, tc.c, s1);
            o2 = uav.sim.run_est(tc.p, tc.c, s2);
            r1 = tc.tailRms(o1);
            r2 = tc.tailRms(o2);
            tc.verifyLessThan(r2, 0.5 * r1);
        end

        function estimateUnbiasedInHover(tc)
            % 悬停末段的位置估计误差均值 < 10 cm
            s = tc.fastCfg(45);
            out = uav.sim.run_est(tc.p, tc.c, s);
            tail = out.t > 22;
            pe = out.est(tail, :) - out.p(tail, :);
            tc.verifyLessThan(max(abs(mean(pe))), 0.10);
        end

        function hoverAccuracyIsSubDecimetre(tc)
            % 光流 + GPS 速度辅助下，60 s 悬停末段 RMS < 0.12 m
            s = tc.fastCfg(60);
            out = uav.sim.run_est(tc.p, tc.c, s);
            tc.verifyLessThan(tc.tailRms(out), 0.12);
        end

        function attitudeEstimateStaysSmall(tc)
            % 姿态估计误差（末段均值）< 2°
            s = tc.fastCfg(45);
            out = uav.sim.run_est(tc.p, tc.c, s);
            tail = out.t > 22;
            attE = zeros(sum(tail), 3);
            qe = out.qEst(tail, :);
            for i = 1:size(qe, 1)
                attE(i, :) = uav.plant.quat2euler(qe(i, :));
            end
            tc.verifyLessThan(max(abs(rad2deg(mean(attE - out.eul(tail, :))))), 2);
        end

        function flowBiasGrowsWithSpeed(tc)
            % 光流的 2% 标度误差是"相对速度"的误差：22 m/s 时等于 0.44 m/s 的假速度，
            % 5 m/s 时只有 0.1 m/s。REPORT_r12 里巡航位置 RMS 因此从 0.713 涨到 4.696 m。
            q = [1;0;0;0];
            mRef = [0.42;0;0.91]; mRef = mRef / norm(mRef);
            rng(11);
            zSlow = uav.sens.flow_sample(q, [5;0;0], 0.03, 1.02);
            rng(11);
            zFast = uav.sens.flow_sample(q, [22;0;0], 0.03, 1.02);
            tc.verifyEqual(zSlow(1) - 5, 0.02*5, 'AbsTol', 0.06);
            tc.verifyEqual(zFast(1) - 22, 0.02*22, 'AbsTol', 0.06);
            tc.verifyGreaterThan(abs(zFast(1) - 22), 3*abs(zSlow(1) - 5));
        end

        function repeatedRunIsIdentical(tc)
            % 固定种子下两次运行逐位相同
            s = tc.fastCfg(10);
            o1 = uav.sim.run_est(tc.p, tc.c, s);
            o2 = uav.sim.run_est(tc.p, tc.c, s);
            tc.verifyEqual(o1.x, o2.x);
        end
    end

    methods (Access = private)
        function s = fastCfg(~, tEnd)
            s = struct('dt', 1e-3, 'ctrlDiv', 1, 'tEnd', tEnd, 'logDiv', 10);
            s.sens = struct('sigmaMag', 0.015, 'useFlow', true, 'sigmaFlow', 0.03, ...
                'bg', [0.02; -0.015; 0.01], 'ba', [0.10; -0.08; 0.06]);
        end

        function r = tailRms(~, out)
            tail = out.t > 0.55 * out.t(end);
            e = out.p(tail, :) - out.ref(tail, :);
            r = sqrt(mean(sum(e.^2, 2)));
        end
    end
end
