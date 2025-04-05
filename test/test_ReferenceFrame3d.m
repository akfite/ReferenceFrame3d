classdef test_ReferenceFrame3d < matlab.unittest.TestCase

    methods (Test)
        function constructors(this)
            a = ReferenceFrame3d();
            this.verifyEqual(a.T, eye(4));
        end

        function static_constructors(this)
        end

        function local2base(this)
            a = ReferenceFrame3d();
            this.verifyEqual(a.T, eye(4));

            % test some 
            a.rotate_euler([45 0 0]);
            vec_out = a.local2base([1/sqrt(2) 1/sqrt(2) 0]);
            this.verifyEqual(vec_out, [0 1 0], 'AbsTol', eps);
        end
    end

end
