classdef tDisturbance < matlab.unittest.TestCase
%TDISTURBANCE 阵风模型与 ESO 抗扰的单元测试。
%   运行：runtests('tests/tDisturbance.m')

    properties
        p
        c
        cfg
        gust
        ref
    end

    methods (TestClassSetup)
        function setupOnce(tc)
            tc.p = uav.cfg.quad();
            tc.c = uav.cfg.ctrl();
            tc.cfg = struct('dt', 1e-3, 'ctrlDiv', 10, 'tEnd', 30, 'logDiv', 10);
            tc.gust = uav.env.wind_dryden(tc.cfg.tEnd, tc.cfg.dt, 10, 2.0, 11, [3; 2; 0]);
            tc.ref = uav.env.hover(0, 0, -5, 0);
        end
    end

    methods (Test)
        function drydenStatisticsMatchInput(tc)
            % 长窗口 + 少量种子：T >> L/V 时样本均值/方差才收敛
            nt = 4;
            Wm = zeros(nt, 3);
            Ws = zeros(nt, 3);
            for i = 1:nt
                g = uav.env.wind_dryden(900, 0.02, 10, 2.0, 500 + i, [3; 2; 0]);
                Wm(i, :) = mean(g.W);
                Ws(i, :) = std(g.W);
            end
            tc.verifyLessThan(max(abs(mean(Wm) - tc.gust.Wmean')), 0.5);
            tc.verifyLessThan(max(abs(mean(Ws) - tc.gust.sigma)), 0.2 * tc.gust.sigma);
        end
        function zeroIntensityGivesMeanWind(tc)
            % sigma = 0 时应恒等于平均风
            g = uav.env.wind_dryden(5, 0.01, 10, 0, 7, [3; 2; 0]);
            tc.verifyEqual(max(max(abs(g.W - repmat(g.Wmean', size(g.W, 1), 1)))), 0);
        end
        function noWindHasNoOffset(tc)
            % 无风时 PID 悬停误差在毫米级
            out = uav.sim.run_wind(tc.p, tc.c, tc.cfg, tc.ref, []);
            e = out.p - out.ref;
            tail = out.t > 15;
            tc.verifyLessThan(sqrt(mean(sum(e(tail, 1:2).^2, 2))), 0.01);
        end

        function windCausesSteadyOffset(tc)
            % 3 m/s 平均风下，纯 PID 的水平误差超过 5 cm
            out = uav.sim.run_wind(tc.p, tc.c, tc.cfg, tc.ref, tc.gust);
            e = out.p - out.ref;
            tail = out.t > 15;
            tc.verifyGreaterThan(sqrt(mean(sum(e(tail, 1:2).^2, 2))), 0.05);
        end

        function esoHalvesTrackingError(tc)
            % ESO 的水平 RMS 应小于 PID 的一半
            cfg1 = tc.cfg; cfg1.useEso = false;
            cfg2 = tc.cfg; cfg2.useEso = true;
            o1 = uav.sim.run_wind(tc.p, tc.c, cfg1, tc.ref, tc.gust);
            o2 = uav.sim.run_wind(tc.p, tc.c, cfg2, tc.ref, tc.gust);
            r1 = tc.tailRms(o1);
            r2 = tc.tailRms(o2);
            tc.verifyLessThan(r2, 0.5 * r1);
        end

        function esoEstimatesWindAcceleration(tc)
            % 稳态扰动加速度应接近 drag*W/m；估计里还含湍流与机体自身运动，容差 35%
            cfg = tc.cfg; cfg.useEso = true;
            out = uav.sim.run_wind(tc.p, tc.c, cfg, tc.ref, tc.gust);
            dTrue = tc.p.drag * tc.gust.Wmean / tc.p.mass;
            dEst = mean(out.dHat(out.t > 15, :))';
            err = norm(dEst - dTrue) / norm(dTrue);
            tc.verifyLessThan(err, 0.35);
        end

        function esoSurvivesMeasurementNoise(tc)
            % 5 cm 量测噪声下仍要明显优于 PID
            cfg1 = tc.cfg; cfg1.useEso = false;
            cfg2 = tc.cfg; cfg2.useEso = true; cfg2.esoNoise = 0.05;
            rng(3, 'twister');
            o1 = uav.sim.run_wind(tc.p, tc.c, cfg1, tc.ref, tc.gust);
            rng(3, 'twister');
            o2 = uav.sim.run_wind(tc.p, tc.c, cfg2, tc.ref, tc.gust);
            tc.verifyLessThan(tc.tailRms(o2), 0.5 * tc.tailRms(o1));
        end

        function esoIsStableWithoutDisturbance(tc)
            % 无风时不应被 ESO 带坏
            cfg = tc.cfg; cfg.useEso = true;
            out = uav.sim.run_wind(tc.p, tc.c, cfg, tc.ref, []);
            tc.verifyLessThan(tc.tailRms(out), 0.01);
        end

        function repeatedRunIsIdentical(tc)
            cfg = tc.cfg; cfg.useEso = true;
            o1 = uav.sim.run_wind(tc.p, tc.c, cfg, tc.ref, tc.gust);
            o2 = uav.sim.run_wind(tc.p, tc.c, cfg, tc.ref, tc.gust);
            tc.verifyEqual(o1.x, o2.x);
        end
    end

    methods (Access = private)
        function r = tailRms(~, out)
            tail = out.t > 15;
            e = out.p(tail, 1:2) - out.ref(tail, 1:2);
            r = sqrt(mean(sum(e.^2, 2)));
        end
    end
end
