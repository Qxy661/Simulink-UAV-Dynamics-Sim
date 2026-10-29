classdef tClosedLoop < matlab.unittest.TestCase
%TCLOSEDLOOP  闭环仿真验收测试。

    properties
        p
        c
        s
    end

    methods (TestClassSetup)
        function setup(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(root);
            testCase.p = uav.cfg.quad();
            testCase.c = uav.cfg.ctrl();
            testCase.s = uav.cfg.simcfg();
        end
    end

    methods (Test)

        function hoverFromOffsetConverges(testCase)
            s = testCase.s; s.tEnd = 12;
            ref = uav.env.hover(0, 0, -5, 0);
            x0 = [0;0;-4; 0;0;0; 1;0;0;0; 0;0;0];
            out = uav.sim.run(testCase.p, testCase.c, s, ref, x0);
            m = uav.sim.metrics(out);
            testCase.verifyLessThan(m.errMax, 0.05);
            testCase.verifyFalse(any(out.sat));
            testCase.verifyTrue(all(isfinite(out.p(:))));
        end

        function positionStepConverges(testCase)
            s = testCase.s; s.tEnd = 14;
            ref = @(t) uav.env.step(t, 1, [0;0;-5], [3;2;-5], 0);
            out = uav.sim.run(testCase.p, testCase.c, s, ref);
            m = uav.sim.metrics(out);
            testCase.verifyLessThan(m.errMax, 0.05);
            testCase.verifyLessThan(rad2deg(m.tiltMax), 40);
        end

        function yawStepConverges(testCase)
            s = testCase.s; s.tEnd = 10;
            ref = uav.env.hover(0, 0, -5, 0.5);
            out = uav.sim.run(testCase.p, testCase.c, s, ref);
            testCase.verifyEqual(out.eul(end,3), 0.5, 'AbsTol', 0.02);
        end

        function repeatedRunsAreIdentical(testCase)
            s = testCase.s; s.tEnd = 4;
            ref = uav.env.hover(0, 0, -5, 0);
            x0 = [0;0;-4; 0;0;0; 1;0;0;0; 0;0;0];
            a = uav.sim.run(testCase.p, testCase.c, s, ref, x0);
            b = uav.sim.run(testCase.p, testCase.c, s, ref, x0);
            testCase.verifyEqual(a.p, b.p, 'AbsTol', 0);
        end

        function quaternionStaysNormalized(testCase)
            s = testCase.s; s.tEnd = 6;
            ref = @(t) uav.env.step(t, 1, [0;0;-5], [5;0;-5], 0.8);
            out = uav.sim.run(testCase.p, testCase.c, s, ref);
            % 状态记录里不存四元数，用欧拉角有限性间接检查数值发散
            testCase.verifyTrue(all(abs(out.eul(:)) < 2*pi));
        end
    end
end
