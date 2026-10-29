classdef tFixedWing < matlab.unittest.TestCase
%TFIXEDWING  固定翼气动、配平、控制器与闭环测试。

    properties
        p
        c
        s
    end

    methods (TestClassSetup)
        function setup(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(root);
            testCase.p = uav.cfg.fw();
            testCase.c = uav.cfg.fwctrl();
            testCase.s = uav.cfg.simcfg();
        end
    end

    methods (Test)

        function trimIsEquilibrium(testCase)
            p = testCase.p;
            tr = uav.plant.fw_trim(p, 18);
            q  = uav.plant.euler2quat(0, tr.alpha, 0);
            Vb = 18 * [cos(tr.alpha); 0; sin(tr.alpha)];
            x  = [0; 0; -100; Vb; q; 0; 0; 0];
            [Fb, Mb] = uav.plant.fw_aero(Vb, [0; 0; 0], [tr.thr; tr.de; 0; 0], p);
            xd = uav.plant.rigid_body(x, Fb, Mb, p);
            testCase.verifyLessThan(norm(xd(4:6)), 1e-9);
            testCase.verifyLessThan(norm(xd(11:13)), 1e-9);
        end

        function trimLiftMatchesWeight(testCase)
            p = testCase.p;
            tr = uav.plant.fw_trim(p, 18);
            testCase.verifyEqual(tr.L, p.mass*p.g, 'RelTol', 0.02);
            testCase.verifyGreaterThan(tr.LD, 10);
            testCase.verifyGreaterThan(tr.thr, 0);
            testCase.verifyLessThan(tr.thr, 0.5);
        end

        function liftIncreasesWithAngleOfAttack(testCase)
            p = testCase.p;
            [F1, ~] = uav.plant.fw_aero([18; 0; 0], [0;0;0], [0.1; 0; 0; 0], p);
            a = 5*pi/180;
            [F2, ~] = uav.plant.fw_aero([18*cos(a); 0; 18*sin(a)], [0;0;0], [0.1; 0; 0; 0], p);
            testCase.verifyLessThan(F2(3), F1(3));
        end

        function elevatorSignIsNoseDown(testCase)
            p = testCase.p;
            [~, M0] = uav.plant.fw_aero([18; 0; 0], [0;0;0], [0.1; 0; 0; 0], p);
            [~, M1] = uav.plant.fw_aero([18; 0; 0], [0;0;0], [0.1; 2*pi/180; 0; 0], p);
            testCase.verifyLessThan(M1(2), M0(2));
        end

        function aileronRollsRight(testCase)
            p = testCase.p;
            [~, M0] = uav.plant.fw_aero([18; 0; 0], [0;0;0], [0.1; 0; 0; 0], p);
            [~, M1] = uav.plant.fw_aero([18; 0; 0], [0;0;0], [0.1; 0; 2*pi/180; 0], p);
            testCase.verifyGreaterThan(M1(1), M0(1));
        end

        function sideslipProducesSideForce(testCase)
            p = testCase.p;
            [F0, ~] = uav.plant.fw_aero([18; 0; 0], [0;0;0], [0.1; 0; 0; 0], p);
            [F1, ~] = uav.plant.fw_aero([18; 1; 0], [0;0;0], [0.1; 0; 0; 0], p);
            testCase.verifyLessThan(F1(2), F0(2));
        end

        function eulerQuatRoundTrip(testCase)
            rng(11);
            for k = 1:10
                e = [(rand-0.5)*0.6; (rand-0.5)*0.6; (rand-0.5)*2.8];
                q = uav.plant.euler2quat(e(1), e(2), e(3));
                testCase.verifyEqual(uav.plant.quat2euler(q), e, 'AbsTol', 1e-10);
            end
        end

        function actuatorStepMatchesFirstOrder(testCase)
            p = testCase.p;
            dt = 1e-3;
            a = uav.plant.actuator_step([1; 0; 0; 0], [0; 0; 0; 0], p, dt);
            expect = 1 - exp(-dt/p.tauT);
            testCase.verifyEqual(a(1), expect, 'AbsTol', 1e-12);
            testCase.verifyEqual(a(2), 0, 'AbsTol', 1e-12);
        end

        function levelFlightHoldsAltitude(testCase)
            testCase.s.tEnd = 10;
            ref = struct('h', 100, 'psi', 0, 'V', 18);
            out = uav.sim.run_fw(testCase.p, testCase.c, testCase.s, ref);
            h = -out.p(:, 3);
            testCase.verifyLessThan(max(abs(h - 100)), 2);
            V = vecnorm(out.v, 2, 2);
            testCase.verifyLessThan(max(abs(V - 18)), 1);
        end

        function climbReachesReference(testCase)
            testCase.s.tEnd = 25;
            p = testCase.p;
            c = testCase.c;
            c.Kth_h = 0.030;
            ref = @(t) struct('h', 100 + 30*(t >= 5), 'psi', 0, 'V', 18);
            out = uav.sim.run_fw(p, c, testCase.s, ref);
            h = -out.p(:, 3);
            testCase.verifyLessThan(abs(h(end) - 130), 0.5);
            testCase.verifyLessThan(max(h), 131.5);
        end

        function headingTurnConverges(testCase)
            testCase.s.tEnd = 40;
            ref = @(t) struct('h', 100, 'psi', (60*pi/180)*(t >= 5), 'V', 18);
            out = uav.sim.run_fw(testCase.p, testCase.c, testCase.s, ref);
            psi = uav.ctrl.wrap_pi(out.eul(end, 3));
            testCase.verifyLessThan(abs(psi - 60*pi/180), 2*pi/180);
            testCase.verifyLessThan(max(abs(rad2deg(out.eul(:,1)))), 40);
        end

        function quaternionStaysNormalized(testCase)
            testCase.s.tEnd = 12;
            ref = @(t) struct('h', 100 + 10*(t >= 3), 'psi', 0, 'V', 18);
            out = uav.sim.run_fw(testCase.p, testCase.c, testCase.s, ref);
            n = vecnorm(out.x(:, 7:10), 2, 2);
            testCase.verifyLessThan(max(abs(n - 1)), 1e-12);
            testCase.verifyTrue(all(isfinite(out.x(:))));
        end

        function repeatedRunIsIdentical(testCase)
            testCase.s.tEnd = 6;
            ref = @(t) struct('h', 105, 'psi', 0.2, 'V', 19);
            a = uav.sim.run_fw(testCase.p, testCase.c, testCase.s, ref);
            b = uav.sim.run_fw(testCase.p, testCase.c, testCase.s, ref);
            testCase.verifyEqual(a.x, b.x);
        end

    end
end
