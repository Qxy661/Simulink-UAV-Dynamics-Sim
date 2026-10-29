classdef tPlant < matlab.unittest.TestCase
%TPLANT  刚体、混控、电机模型单元测试。

    properties
        p
    end

    methods (TestClassSetup)
        function addPath(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(root);
            testCase.p = uav.cfg.quad();
        end
    end

    methods (Test)

        function dcmIsOrthonormal(testCase)
            rng(1);
            for k = 1:20
                q = randn(4, 1);
                R = uav.plant.q2dcm(q);
                testCase.verifyEqual(R'*R, eye(3), 'AbsTol', 1e-12);
                testCase.verifyEqual(det(R), 1, 'AbsTol', 1e-12);
            end
        end

        function eulerRoundTrip(testCase)
            rng(2);
            for k = 1:20
                e = [(rand-0.5)*pi; (rand-0.5)*1.2; (rand-0.5)*pi];
                q = eulerToQuat(e);
                e2 = uav.plant.quat2euler(q);
                testCase.verifyEqual(e2, e, 'AbsTol', 1e-10);
            end
        end

        function quatMultiplicationIsIdentity(testCase)
            q = [0.9; 0.1; -0.3; 0.2];
            q = q/norm(q);
            testCase.verifyEqual(uav.plant.qmul([1;0;0;0], q), q, 'AbsTol', 1e-14);
            testCase.verifyEqual(uav.plant.qmul(q, [1;0;0;0]), q, 'AbsTol', 1e-14);
        end

        function hoverIsEquilibrium(testCase)
            p = testCase.p;
            x = [0;0;0; 0;0;0; 1;0;0;0; 0;0;0];
            xd = uav.plant.rigid_body(x, [0;0;-p.mass*p.g], [0;0;0], p);
            testCase.verifyEqual(xd, zeros(13, 1), 'AbsTol', 1e-12);
        end

        function freeFallAcceleratesDown(testCase)
            p = testCase.p;
            x = [0;0;0; 0;0;0; 1;0;0;0; 0;0;0];
            xd = uav.plant.rigid_body(x, [0;0;0], [0;0;0], p);
            testCase.verifyEqual(xd(4:6), [0;0;p.g], 'AbsTol', 1e-12);
        end

        function torqueProducesAngularAcceleration(testCase)
            p = testCase.p;
            x = [0;0;0; 0;0;0; 1;0;0;0; 0;0;0];
            M = [0.2; -0.1; 0.05];
            xd = uav.plant.rigid_body(x, [0;0;0], M, p);
            testCase.verifyEqual(xd(11:13), p.I \ M, 'AbsTol', 1e-12);
        end

        function mixerRoundTrip(testCase)
            p = testCase.p;
            TM = [14.7; 0.5; -0.3; 0.1];
            w = uav.plant.mixer(TM, p);
            [Fb, Mb] = uav.plant.quad_wrench(w, [0;0;0], p);
            testCase.verifyEqual(-Fb(3), TM(1), 'AbsTol', 1e-9);
            testCase.verifyEqual(Mb, TM(2:4), 'AbsTol', 1e-12);
        end

        function mixerRejectsNegativeThrust(testCase)
            p = testCase.p;
            w = uav.plant.mixer([0; 5; 0; 0], p);
            testCase.verifyTrue(all(isfinite(w)));
            testCase.verifyTrue(all(w >= 0));
        end

        function equalRotorSpeedsGiveNoMoment(testCase)
            p = testCase.p;
            w = repmat(p.hoverW, 4, 1);
            [~, Mb] = uav.plant.quad_wrench(w, [0;0;0], p);
            testCase.verifyEqual(Mb, zeros(3, 1), 'AbsTol', 1e-10);
        end

        function yawMomentSign(testCase)
            p = testCase.p;
            w = p.hoverW*[1.05; 0.95; 1.05; 0.95];
            [~, Mb] = uav.plant.quad_wrench(w, [0;0;0], p);
            % 1/3 为逆时针，增大 1/3 转速产生正向偏航力矩
            testCase.verifyGreaterThan(Mb(3), 0);
        end

        function motorStepReaches63Percent(testCase)
            p = testCase.p;
            w0 = 0; wcmd = 1000; tau = p.tauM;
            w = uav.plant.motor_step(w0, wcmd, p, tau);
            testCase.verifyEqual(w/wcmd, 1 - exp(-1), 'AbsTol', 1e-12);
        end

        function motorStepRespectsLimit(testCase)
            p = testCase.p;
            w = uav.plant.motor_step(0, 5000, p, 10);
            testCase.verifyEqual(w, p.wMax, 'AbsTol', 1e-12);
        end

        function dragOpposesVelocity(testCase)
            p = testCase.p;
            [Fb, ~] = uav.plant.quad_wrench(repmat(p.hoverW,4,1), [1;0;0], p);
            testCase.verifyLessThan(Fb(1), 0);
        end
    end
end

function q = eulerToQuat(e)
phi = e(1)/2; th = e(2)/2; ps = e(3)/2;
q = [cos(phi)*cos(th)*cos(ps) + sin(phi)*sin(th)*sin(ps);
     sin(phi)*cos(th)*cos(ps) - cos(phi)*sin(th)*sin(ps);
     cos(phi)*sin(th)*cos(ps) + sin(phi)*cos(th)*sin(ps);
     cos(phi)*cos(th)*sin(ps) - sin(phi)*sin(th)*cos(ps)];
q = q/norm(q);
end
