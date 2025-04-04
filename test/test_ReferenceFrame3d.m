classdef test_ReferenceFrame3d < matlab.unittest.TestCase

    methods (Test)
        function constructors(this)
            a = ReferenceFrame3d();
            this.verifyEqual(a.T, eye(4));
        end

        function static_constructors(this)
        end
    end

end
