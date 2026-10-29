classdef tController < matlab.unittest.TestCase
%TCONTROLLER  级联控制器单元测试。

    properties
        p
        c
    end

    methods (TestClassSetup)
        function setup(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(root);
            testCase.p = uav.cfg.quad();
            testCase.c = uav.cfg.ctrl();
        end
    end

    methods (Access = private)
        function st = state(testCase) %#ok<MANU>
            st = struct('ip', [0;0;0], 'ia', [0;0;0]);
        end

        function x = hoverState(testCase)
            x = [0;0;-5; 0;0;0; 1;0;0;0; 0;0;0];
        end
    end

    methods (Test)

        function hoverCommandIsWeight(testCase)
            ref = uav.env.hover(0, 0, -5, 0);
            [T, M, ~] = uav.ctrl.cascade(testCase.hoverState(), ref, ...
                testCase.p, testCase.c, testCase.state(), 0.01);
            testCase.verifyEqual(T, testCase.p.mass*testCase.p.g, 'RelTol', 1e-9);
            testCase.verifyEqual(M, zeros(3,1), 'AbsTol', 1e-12);
        end

        function northErrorCommandNoseDown(testCase)
            ref = uav.env.hover(2, 0, -5, 0);
            [~, M, ~] = uav.ctrl.cascade(testCase.hoverState(), ref, ...
                testCase.p, testCase.c, testCase.state(), 0.01);
            % 机头向下（负俯仰）才能产生向北推力
            testCase.verifyLessThan(M(2), 0);
        end

        function eastErrorCommandRollRight(testCase)
            ref = uav.env.hover(0, 2, -5, 0);
            [~, M, ~] = uav.ctrl.cascade(testCase.hoverState(), ref, ...
                testCase.p, testCase.c, testCase.state(), 0.01);
            testCase.verifyGreaterThan(M(1), 0);
        end

        function yawErrorProducesYawMoment(testCase)
            ref = uav.env.hover(0, 0, -5, 0.3);
            [~, M, ~] = uav.ctrl.cascade(testCase.hoverState(), ref, ...
                testCase.p, testCase.c, testCase.state(), 0.01);
            testCase.verifyGreaterThan(M(3), 0);
        end

        function thrustIsCapped(testCase)
            ref = uav.env.hover(0, 0, 50, 0);   % 50 m 向下，远超能力
            [T, ~, ~] = uav.ctrl.cascade(testCase.hoverState(), ref, ...
                testCase.p, testCase.c, testCase.state(), 0.01);
            testCase.verifyLessThanOrEqual(T, testCase.c.maxT);
        end

        function momentIsCapped(testCase)
            ref = uav.env.hover(0, 0, -5, 3.0);
            [~, M, ~] = uav.ctrl.cascade(testCase.hoverState(), ref, ...
                testCase.p, testCase.c, testCase.state(), 0.01);
            testCase.verifyLessThanOrEqual(abs(M(3)), testCase.c.maxM(3) + 1e-12);
        end

        function integralWindsUpBounded(testCase)
            ref = uav.env.hover(0, 0, -50, 0);
            st = testCase.state();
            for k = 1:500
                [~, ~, st] = uav.ctrl.cascade(testCase.hoverState(), ref, ...
                    testCase.p, testCase.c, st, 0.01);
            end
            testCase.verifyLessThanOrEqual(abs(st.ip(3)), testCase.c.iMax(3) + 1e-12);
            testCase.verifyEqual(st.ip(1:2), [0; 0], 'AbsTol', 1e-12);
        end
    end
end
