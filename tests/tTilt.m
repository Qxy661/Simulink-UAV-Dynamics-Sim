classdef tTilt < matlab.unittest.TestCase
%TTILT  倾转四旋翼：混控、分配、短舱调度与闭环测试。

    properties
        p
        c
        s
    end

    methods (TestClassSetup)
        function setup(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(root);
            testCase.p = uav.cfg.tilt();
            testCase.c = uav.cfg.tiltctrl();
            testCase.s = uav.cfg.simcfg();
        end
    end

    methods (Access = private)
        function x = hoverState(~, p)
            q = uav.plant.euler2quat(0, 0, 0);
            x = [0; 0; -p.htrim; 0; 0; 0; q; 0; 0; 0; 0; p.hoverW*ones(4,1); 0; 0; 0];
        end
    end

    methods (Test)

        function hoverIsEquilibrium(testCase)
            p = testCase.p;
            x = testCase.hoverState(p);
            [Fb, Mb] = uav.plant.tilt_wrench(x, x(15:18), x(19:21), x(14), p);
            xd = uav.plant.rigid_body(x(1:13), Fb, Mb, p);
            testCase.verifyLessThan(norm(xd(4:6)), 1e-9);
            testCase.verifyLessThan(norm(xd(11:13)), 1e-12);
        end

        function mixRoundTrip(testCase)
            p = testCase.p;
            U = [78.5; 0.6; -0.4; 0.2];
            om = uav.plant.tilt_mix(U(1), U(2), U(3), U(4), p);
            T  = p.kt * om.^2;
            U1 = sum(T);
            U2 = -sum(p.ry(:) .* T);
            U3 = sum(p.rx(:) .* T);
            U4 = sum(p.spin(:) .* T);
            testCase.verifyEqual([U1; U2; U3; U4], U, 'RelTol', 1e-10);
        end

        function betaZeroGivesQuadRoll(testCase)
            p = testCase.p;
            x = testCase.hoverState(p);
            % 左右两侧差动：1/2 在右、3/4 在左
            om = p.hoverW * [1.1; 1.1; 0.9; 0.9];
            [~, Mb0] = uav.plant.tilt_wrench(x, om, x(19:21), 0, p);
            [~, Mb9] = uav.plant.tilt_wrench(x, om, x(19:21), pi/2, p);
            % beta=0 时转速差动给出滚转力矩，beta=90 时该通道基本失效
            testCase.verifyGreaterThan(abs(Mb0(1)), 1.0);
            testCase.verifyLessThan(abs(Mb9(1)), 0.1*abs(Mb0(1)));
        end

        function yawMomentFromReactionTorque(testCase)
            p = testCase.p;
            x = testCase.hoverState(p);
            om = p.hoverW * [1.05; 0.95; 1.05; 0.95];
            [~, Mb] = uav.plant.tilt_wrench(x, om, x(19:21), 0, p);
            testCase.verifyGreaterThan(Mb(3), 0);
        end

        function betaScheduleIsMonotone(testCase)
            p = testCase.p;
            b = zeros(1,21);
            for i = 1:numel(b)
                refV = p.V1 + (p.V2-p.V1)*(i-1)/20;
                sc = uav.ctrl.tilt_schedule(refV, p);
                b(i) = sc.beta;
            end
            testCase.verifyTrue(all(diff(b) >= -1e-12));
            testCase.verifyEqual(b(1), p.betaMin, 'AbsTol', 1e-12);
            testCase.verifyEqual(b(end), p.betaMax, 'AbsTol', 1e-12);
        end

        function rotorTakesAllAtLowSpeed(testCase)
            p = testCase.p;
            c = testCase.c;
            M = [1.0; -1.0; 0.2];
            [omLow, actLow] = uav.ctrl.tilt_alloc(M, p.mass*p.g, 0, 0, 0, 0, p, c);
            testCase.verifyGreaterThan(max(omLow), p.hoverW*0.9);
            testCase.verifyLessThan(norm(actLow), 1e-9);
            % k=1 时舵面接管，旋翼只剩竖直支撑
            [omHigh, actHigh] = uav.ctrl.tilt_alloc(M, p.mass*p.g, p.mass*p.g, 1, pi/2, 200, p, c);
            testCase.verifyGreaterThan(abs(actHigh(2)), 1e-3);
            testCase.verifySize(omHigh, [4 1]);
        end

        function surfaceTakesOverAtHighSpeed(testCase)
            p = testCase.p;
            c = testCase.c;
            M = [2.0; 0; 0];
            [~, act, ~] = uav.ctrl.tilt_alloc(M, p.mass*p.g, p.mass*p.g, 1, pi/2, 200, p, c);
            testCase.verifyGreaterThan(abs(act(2)), 1e-3);
        end

        function missionHoldsAltitude(testCase)
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 45; s.logDiv = 50;
            out = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
            h = -out.p(:,3);
            % 加速段（5–20 s）会先爬 4 m 再被高度积分拉回，末段才是稳态指标
            tail = out.t > 35;
            testCase.verifyLessThan(max(abs(h - p.htrim)), 5.0);
            testCase.verifyLessThan(max(abs(h(tail) - p.htrim)), 0.5);
        end

        function missionReachesCruiseTilt(testCase)
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 45; s.logDiv = 100;
            out = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
            V  = vecnorm(out.v, 2, 2);
            bt = 180/pi*out.beta;
            testCase.verifyGreaterThan(max(bt), 85);
            testCase.verifyGreaterThan(V(end), 21);
            testCase.verifyLessThan(abs(180/pi*max(out.eul(:,2))), 20);
        end

        function missionTracksReference(testCase)
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 45; s.logDiv = 100;
            out = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
            ep = vecnorm(out.p - out.ref(:,1:3), 2, 2);
            testCase.verifyLessThan(ep(end), 2.0);
        end

        function backTransitionRespectsEnergyBudget(testCase)
            % 减速段的高度代价是能量账，不是控制器指标：8 kg 从 16 m/s 停下
            % 需要耗散 1024 J，阻力在刹车距离上只能吃掉一部分，余下变成爬升。
            % 这里检查实测峰值落在 [0.5, 1.6] * V^2/(2g) 区间（上界放宽到 1.6 是因为
            % 峰值里还叠着任务本身要求的 12 m 下降，阻力节省比例在 8%-45% 之间浮动）：
            % 低于下界说明有人凭空消掉了动能，高于上界说明控制器在白送高度。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 70; s.logDiv = 50;
            Vmax = 16;
            ref = @(t) uav.env.tilt_ref_bt(t, Vmax, 20, 8, 15);
            out = uav.sim.run_tilt(p, c, s, ref);
            dh = -out.p(:,3) - out.ref(:,5);
            peak = max(abs(dh));
            eBound = Vmax^2/(2*p.g);
            testCase.verifyGreaterThan(peak, 0.5*eBound);
            testCase.verifyLessThan(peak, 1.6*eBound);
        end

        function deadRotorIsNotCommanded(testCase)
            % 一台电机完全失效时分配器不能给它任何转速指令：
            % B*diag(act) 的第 1 列为零，伪逆解的 T(1) 必须是 0。
            p = testCase.p;
            act = [0; 1; 1; 1];
            [om, ~, T] = uav.plant.tilt_mix_fault(20, 0.2, 0.1, 0, p, act);
            testCase.verifyEqual(om(1), 0);
            testCase.verifyEqual(T(1), 0);
            testCase.verifyGreaterThan(sum(p.kt*om.^2), 1);
        end

        function faultHardReducesDrift(testCase)
            % 降效 35% 时把坏电机按完全失效处理，巡航段位置漂移应压在 3 m 内。
            % 按真实降效分配会得到约 4.8 m（见 research/REPORT_r4_fault.md）。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 45; s.logDiv = 50;
            s.fault = struct('t', 25, 'k', 1, 'eta', 0.35);
            s.faultAware = true;
            s.faultHard  = true;
            out = uav.sim.run_tilt(p, c, s, @(t) uav.env.tilt_ref_v(t, 22));
            post = out.t > 25;
            ep = vecnorm(out.p(post,1:2) - out.ref(post,1:2), 2, 2);
            testCase.verifyLessThan(max(ep), 3.0);
            testCase.verifyLessThan(max(abs(-out.p(post,3) - out.ref(post,5))), 2.0);
        end

        function faultDetectorLatency(testCase)
            % 残差检测：正常电机残差 0、失效电机残差 495 rad/s，3 拍后判死。
            d = struct('cnt', zeros(4,1), 'dead', false(4,1), 'th', 60, 'hold', 3);
            wCmd = 495*ones(4,1);
            wMeas = wCmd; wMeas(1) = 0;
            for k = 1:2
                [d, idx] = uav.est.rotor_fault_detect(wCmd, wMeas, d);
                testCase.verifyEmpty(idx);
            end
            [d, idx] = uav.est.rotor_fault_detect(wCmd, wMeas, d);
            testCase.verifyEqual(idx, 1);
            testCase.verifyTrue(d.dead(1));
            testCase.verifyFalse(any(d.dead(2:4)));
        end

        function detectedFaultMatchesOracle(testCase)
            % 只靠残差检测（不知道 η）与先知先觉的漂移差应小于 0.3 m。
            p = testCase.p; c = testCase.c; s0 = testCase.s;
            s0.tEnd = 35; s0.logDiv = 50; s0.faultAware = true;
            s0.fault = struct('t', 25, 'k', 1, 'eta', 0.35);
            ref = @(t) uav.env.tilt_ref_v(t, 22);
            s1 = s0; s1.faultHard = true;
            s2 = s0; s2.detect = true;
            a = uav.sim.run_tilt(p, c, s1, ref);
            b = uav.sim.run_tilt(p, c, s2, ref);
            post = a.t > 25;
            ea = max(vecnorm(a.p(post,1:2) - a.ref(post,1:2), 2, 2));
            eb = max(vecnorm(b.p(post,1:2) - b.ref(post,1:2), 2, 2));
            testCase.verifyLessThan(abs(ea - eb), 0.3);
            testCase.verifyLessThan(eb, 3.0);
        end

        function thrustMarginFailsDesignRule(testCase)
            % 单桨失效悬停存活要求单台最大推力 >= mg/2（裕度 2）。
            % 本配置 wMax=1100 只有 1.60，是已知设计取舍。
            m = uav.cfg.thrust_margin(testCase.p);
            testCase.verifyGreaterThan(m, 1.5);
            testCase.verifyLessThan(m, 2.0);
            need = sqrt(2) * testCase.p.hoverW;
            testCase.verifyGreaterThan(need, testCase.p.wMax);
            testCase.verifyLessThan(abs(need - 1229), 2);
        end

        function hoverSurvivesWithAdequateMargin(testCase)
            % 把 wMax 提到 1300（裕度 2.24）后，悬停断桨不再翻扣：
            % 高度掉不到 1 m，漂移 < 5 m，滚转 < 45°。
            p = testCase.p;
            p.wMax = 1300;
            c = testCase.c; s = testCase.s;
            s.tEnd = 25; s.logDiv = 20;
            s.alloc = 'rankrpi'; s.faultAware = true; s.faultHard = true;
            s.fault = struct('t', 8, 'k', 1, 'eta', 0);
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            out = uav.sim.run_tilt(p, c, s, ref);
            post = out.t > 8;
            hMin  = min(-out.p(post, 3));
            drift = max(vecnorm(out.p(post, 1:2), 2, 2));
            roll  = max(abs(rad2deg(out.eul(post, 1))));
            testCase.verifyGreaterThan(hMin, 19.0);
            testCase.verifyLessThan(drift, 5.0);
            testCase.verifyLessThan(roll, 45);
        end

        function emergencyLandingBoundedAttitude(testCase)
            % 裕度 1.60 的机体断桨后落不到地面上（0.8 m/s 指令只沉 0.25 m/s），
            % 但 1.6 m/s 指令能接地。姿态阻尼从 [1;1;0.8] 调到 [4;4;2] 之后，
            % 接地变得平缓：垂直速度 0.09 m/s、倾角 0.1°（r17 之前是 0.58 m/s、
            % 5.6°）。漂移仍有约 178 m，来自偏航秩亏损，见 REPORT_r7_emergency.md。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 40; s.logDiv = 10;
            s.alloc = 'rankrpi'; s.faultAware = true; s.faultHard = true;
            s.fault = struct('t', 8, 'k', 1, 'eta', 0);
            s.emergency = true; s.emergencyRate = 1.6;
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            out = uav.sim.run_tilt(p, c, s, ref);
            h = -out.p(:, 3);
            k = find(h <= 0.05, 1);
            testCase.verifyNotEmpty(k);
            vz   = -out.v(k, 3);
            tilt = max(abs(rad2deg(out.eul(k, 1:2))));
            testCase.verifyGreaterThan(vz, 0.02);
            testCase.verifyLessThan(vz, 1.5);
            testCase.verifyLessThan(tilt, 15);
        end

        function hoverRecoversFromOffset(testCase)
            % r17：位置环增益不变、把姿态阻尼 [1;1;0.8] 换成 [4;4;2] 之前，
            % 1 m 初始位置偏差会进 0.83 m / 俯仰 18° 的极限环（任何扰动都收敛不了）。
            % 现在 2 m 偏差 15 s 内回到 0.4 m 以内。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 20; s.logDiv = 10;
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            x0 = [2;0;-20; zeros(3,1); 1;0;0;0; zeros(3,1); p.betaMin; ...
                  p.hoverW*ones(4,1); zeros(3,1)];
            out = uav.sim.run_tilt(p, c, s, ref, x0);
            m = out.t > 15;
            testCase.verifyLessThan(max(vecnorm(out.p(m,1:2), 2, 2)), 0.6);
            testCase.verifyLessThan(max(abs(rad2deg(out.eul(m,2)))), 8);
        end

        function hoverHoldsModerateWind(testCase)
            % 2 m/s 恒定北风：收敛到 0.32 m、俯仰 2.2°；4 m/s 已守不住（漂 64 m）。
            % 不开 ESO 时悬停抗风上界约 2 m/s，见 REPORT_r13_hover_robustness.md。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 25; s.logDiv = 10; s.windFn = @(t) [2;0;0];
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            out = uav.sim.run_tilt(p, c, s, ref);
            m = out.t > 5;
            testCase.verifyLessThan(max(vecnorm(out.p(m,1:2), 2, 2)), 1.0);
        end

        function reducedWingImprovesHoverWind(testCase)
            % r19：6 m/s 侧风翻扣不是补偿或增益问题，是机翼在 20° 倾角下给出
            % 约 76 N 升力（与重力同量级），位置环越追越抬头的正反馈。
            % 同一控制器、同一 ESO，只把 S 从 1.6 降到 0.8，6 m/s 就守得住。
            % 见 REPORT_r15_hover_wing_wind.md。
            p = testCase.p; c = testCase.c; s = testCase.s;
            p.S = 0.8;
            s.tEnd = 20; s.logDiv = 10; s.windFn = @(t) [6;0;0];
            s.useEso = true; s.woEso = 8;
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            out = uav.sim.run_tilt(p, c, s, ref);
            m = out.t > 10;
            testCase.verifyLessThan(max(vecnorm(out.p(m,1:2), 2, 2)), 1.5);
            testCase.verifyLessThan(max(max(abs(rad2deg(out.eul(m,1:2))))), 10);
        end

        function esoExtendsTiltHoverWind(testCase)
            % r18：期望加速度上接 ESO 扰动补偿，5 m/s 侧风下末段误差 0.57 m；
            % 不开 ESO 时同样的风会漂 174.9 m 并翻扣。6 m/s 时旋翼饱和，仍会翻。
            % 见 REPORT_r14_tilt_wind_eso.md。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 20; s.logDiv = 10; s.windFn = @(t) [5;0;0];
            s.useEso = true; s.woEso = 8;
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            out = uav.sim.run_tilt(p, c, s, ref);
            m = out.t > 10;
            testCase.verifyLessThan(max(vecnorm(out.p(m,1:2), 2, 2)), 1.5);
            testCase.verifyLessThan(max(max(abs(rad2deg(out.eul(m,1:2))))), 10);
        end

        function cruiseSurvivesDiagonalDoubleFailure(testCase)
            % 22 m/s 巡航、β=90°：对角两台失效后剩下的仍是对角一对，
            % 推力与滚转能配平，舵面接管俯仰/偏航。水平偏差 0.3 m，
            % 高度 0.27 m。见 REPORT_r8_multifault.md。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 45; s.logDiv = 20;
            s.alloc = 'rankrpi'; s.faultAware = true; s.faultHard = true;
            s.fault = struct('t', 30, 'k', [1 3], 'eta', [0; 0]);
            ref = @(t) uav.env.tilt_ref_v(t, 22);
            out = uav.sim.run_tilt(p, c, s, ref);
            post = out.t > 30;
            errH = vecnorm(out.p(:, 1:2) - out.ref(:, 1:2), 2, 2);
            dh   = -out.p(:, 3) - out.ref(:, 5);
            testCase.verifyLessThan(max(errH(post)), 3.0);
            testCase.verifyLessThan(max(abs(dh(post))), 2.0);
        end

        function hoverTumblesWithAdjacentDoubleFailure(testCase)
            % 悬停断相邻两台：滚转只剩单边力矩，机体翻扣（倾角接近 180°）。
            % 这条测试是"不要指望悬停双桨失效"的守门人。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 25; s.logDiv = 20;
            s.alloc = 'rankrpi'; s.faultAware = true; s.faultHard = true;
            s.fault = struct('t', 8, 'k', [1 2], 'eta', [0; 0]);
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            out = uav.sim.run_tilt(p, c, s, ref);
            post = out.t > 8;
            tilt = max(abs(rad2deg(out.eul(post, 1:2))), [], 'all');
            testCase.verifyGreaterThan(tilt, 90);
        end

        function transitionSingleFailureIsBenign(testCase)
            % 过渡段（β≈45°、空速 11 m/s）断 1 台：机翼已经能提供大部分升力，
            % 三台旋翼只需补前向推力，结果与无故障几乎一样（REPORT_r9）。
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 45; s.logDiv = 20;
            s.alloc = 'rankrpi'; s.faultAware = true; s.faultHard = true;
            s.fault = struct('t', 11, 'k', 1, 'eta', 0);
            ref = @(t) uav.env.tilt_ref_v(t, 22);
            out = uav.sim.run_tilt(p, c, s, ref);
            post = out.t > 11;
            errH = vecnorm(out.p(:, 1:2) - out.ref(:, 1:2), 2, 2);
            dh   = -out.p(:, 3) - out.ref(:, 5);
            testCase.verifyLessThan(max(errH(post)), 5.0);
            testCase.verifyLessThan(max(abs(dh(post))), 8.0);
        end

        function marginTwoThresholdMatchesAnalytic(testCase)
            % 裕度 2 的临界转速由 maxT = mg/2 反解；这条测试锁住判据本身，
            % 免得以后改了 kt 或 mass 还拿旧结论说话。
            p = testCase.p;
            wNeed = sqrt(p.mass * p.g / 2 / p.kt);
            testCase.verifyEqual(p.wMax, wNeed, 'RelTol', 0.12);
            testCase.verifyLessThan(p.wMax, wNeed, ...
                '本机 wMax 应低于临界值（已知不合格）');
        end

        function rotorRadiusScalingBuysThrust(testCase)
            % 保持桨尖速度不变时 Tmax ∝ R^2：半径 +11.7% 就能补上 25% 推力，
            % 且转速下降。这条是 r14 的选型结论的可执行版本。
            p = testCase.p;
            needT = p.mass * p.g / 2;
            R0 = 0.30;
            Rneed = R0 * sqrt(needT / (p.kt * p.wMax^2));
            testCase.verifyGreaterThan(Rneed, R0);
            testCase.verifyLessThan(Rneed / R0, 1.15);
        end

        function yawTrimReducesSpin(testCase)
            % 差动短舱（三约束联解）能把单桨失效后的自转压下去：
            % 偏航角速度 RMS 从 0.38 降到 0.18 rad/s 量级。见 REPORT_r11。
            p = testCase.p; c = testCase.c;
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            s = uav.cfg.simcfg(); s.tEnd = 25; s.logDiv = 20;
            s.alloc = 'rankrpi'; s.faultAware = true; s.faultHard = true;
            s.fault = struct('t', 8, 'k', 1, 'eta', 0);
            s.yawTrim = struct('enable', true, 'kp', 0.8, 'kd', 0.25, ...
                               'maxTrim', 12*pi/180);
            out = uav.sim.run_tilt(p, c, s, ref);
            post = out.t > 8;
            rmsR = sqrt(mean(out.w(post, 3).^2));
            testCase.verifyLessThan(rmsR, 0.25);
        end

        function yawTrimDoesNotFixDrift(testCase)
            % 负结果守门：修好自转之后位置漂移反而变大（自转曾把漂移平均掉）。
            % 若哪天这条失败，说明控制结构变了，REPORT_r11 的结论要重写。
            p = testCase.p; c = testCase.c;
            ref = @(t) struct('p', [0;0;-20], 'v', zeros(3,1), 'a', zeros(3,1), ...
                              'psi', 0, 'h', 20, 'V', 0);
            s0 = uav.cfg.simcfg(); s0.tEnd = 25; s0.logDiv = 20;
            s0.alloc = 'rankrpi'; s0.faultAware = true; s0.faultHard = true;
            s0.fault = struct('t', 8, 'k', 1, 'eta', 0);
            out0 = uav.sim.run_tilt(p, c, s0, ref);
            drift0 = max(vecnorm(out0.p(out0.t > 8, 1:2), 2, 2));

            s1 = s0;
            s1.yawTrim = struct('enable', true, 'kp', 0.8, 'kd', 0.25, ...
                                'maxTrim', 12*pi/180);
            out1 = uav.sim.run_tilt(p, c, s1, ref);
            drift1 = max(vecnorm(out1.p(out1.t > 8, 1:2), 2, 2));

            testCase.verifyGreaterThan(drift1, drift0);
        end

        function energyProfileDecelTracksReference(testCase)
            % 沿能量轨迹减速：峰值偏差要明显小于定高方案的 23.8 m，
            % 末端还要贴得住参考高度。见 research/REPORT_r16_energy_profile.md。
            p = testCase.p; c = testCase.c;
            s = testCase.s; s.tEnd = 60; s.logDiv = 20; s.useEso = false;
            ref = @(t) uav.env.tilt_ref_energy(t, 22, 20, 0.7);
            out = uav.sim.run_tilt(p, c, s, ref);
            h = -out.p(:, 3);
            hRef = arrayfun(@(k) ref(out.t(k)).h, 1:numel(out.t)).';
            dec = out.t >= 30 & out.t <= 50;
            hov = out.t > 52;
            testCase.verifyLessThan(max(abs(h(dec) - hRef(dec))), 15);
            testCase.verifyLessThan(max(abs(h(hov) - hRef(hov))), 1.5);
        end

        function esoReducesPitchInTransitionWind(testCase)
            % 转捩段 4 m/s 侧风：ESO 不改善轨迹（9.65→9.36 m），
            % 但把俯仰峰值从 18.2° 压到 8.4°。见 REPORT_r17_transition_wind.md。
            p = testCase.p; c = testCase.c;
            ref = @uav.env.tilt_ref;
            s0 = testCase.s; s0.tEnd = 45; s0.logDiv = 40;
            s0.useEso = false; s0.windFn = @(t) [4; 0; 0];
            out0 = uav.sim.run_tilt(p, c, s0, ref);
            pitch0 = max(abs(rad2deg(out0.eul(out0.t >= 5 & out0.t <= 45, 2))));

            s1 = s0; s1.useEso = true;
            out1 = uav.sim.run_tilt(p, c, s1, ref);
            pitch1 = max(abs(rad2deg(out1.eul(out1.t >= 5 & out1.t <= 45, 2))));

            testCase.verifyLessThan(pitch1, 0.7*pitch0);
        end

        function repeatedRunIsIdentical(testCase)
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 5; s.logDiv = 100;
            a = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
            b = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
            testCase.verifyEqual(a.x, b.x);
        end
    end
end
