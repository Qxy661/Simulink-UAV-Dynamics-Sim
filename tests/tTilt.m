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

        function repeatedRunIsIdentical(testCase)
            p = testCase.p; c = testCase.c; s = testCase.s;
            s.tEnd = 5; s.logDiv = 100;
            a = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
            b = uav.sim.run_tilt(p, c, s, @uav.env.tilt_ref);
            testCase.verifyEqual(a.x, b.x);
        end
    end
end
