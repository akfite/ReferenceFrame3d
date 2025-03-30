classdef test_ReferenceFrame3d < matlab.unittest.TestCase
    %TEST_REFERENCEFRAME3D Unit tests for the ReferenceFrame3d class.
    %   Includes tests for construction, setup, static methods, math operations,
    %   coordinate transformations, property access, graphics, and mixins.
    %   This version removes dependencies on MATLAB toolboxes.

    properties (Constant)
        AbsTol = 1e-12; % Absolute tolerance for floating-point comparisons
        RelTol = 1e-10; % Relative tolerance for floating-point comparisons
    end

    properties
        % Common test objects - initialized in TestMethodSetup
        IdentityFrame % ReferenceFrame3d object representing identity transform
        RotatedFrame % ReferenceFrame3d object with only rotation
        TranslatedFrame % ReferenceFrame3d object with only translation
        GeneralFrame % ReferenceFrame3d object with rotation and translation
        TestFig % Figure handle for graphics tests
        TestAx % Axes handle for graphics tests
    end

    % ================== Local Helper Functions (No Toolbox) ==================
    methods (Static, Access = private)
        function R = local_eul2rotm(eul, sequence)
            %LOCAL_EUL2ROTM Convert Euler angles to rotation matrix (ZYX sequence only).
            arguments
                eul(1,3) double % [roll, pitch, yaw] in radians
                sequence(1,:) char = 'ZYX' % Only ZYX supported here
            end
            if ~strcmpi(sequence, 'ZYX')
                error('test_ReferenceFrame3d:local_eul2rotm', 'Only ZYX sequence supported by local helper.');
            end
            
            roll = eul(1); pitch = eul(2); yaw = eul(3);
            
            cy = cos(yaw); sy = sin(yaw);
            cp = cos(pitch); sp = sin(pitch);
            cr = cos(roll); sr = sin(roll);

            R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
                  sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
                  -sp,   cp*sr,            cp*cr];
        end

        function T = local_eul2tform(eul, sequence)
             %LOCAL_EUL2TFORM Convert Euler angles to homogeneous transform (ZYX sequence only).
             arguments
                eul(1,3) double % [roll, pitch, yaw] in radians
                sequence(1,:) char = 'ZYX' % Only ZYX supported here
            end
             R = test_ReferenceFrame3d.local_eul2rotm(eul, sequence);
             T = eye(4);
             T(1:3, 1:3) = R;
        end

        function eul = local_rotm2eul(R, sequence)
            %LOCAL_ROTM2EUL Convert rotation matrix to Euler angles (ZYX sequence only).
             arguments
                R(3,3) double
                sequence(1,:) char = 'ZYX' % Only ZYX supported here
            end
            if ~strcmpi(sequence, 'ZYX')
                 error('test_ReferenceFrame3d:local_rotm2eul', 'Only ZYX sequence supported by local helper.');
            end

            % Handle gimbal lock: if cos(pitch) is close to 0
            if abs(R(3,1)) > (1 - 1e-12) % Pitch is +/- 90 degrees
                yaw = 0; % Convention: set yaw to 0
                if R(3,1) < 0 % Pitch is +90
                    pitch = pi/2;
                    roll = atan2(R(1,2), R(2,2));
                else % Pitch is -90
                    pitch = -pi/2;
                    roll = -atan2(R(1,2), R(2,2)); % Note the sign change convention might differ
                end
            else
                pitch = asin(-R(3,1)); % Calculate pitch first
                cp = cos(pitch);
                roll = atan2(R(3,2)/cp, R(3,3)/cp);
                yaw = atan2(R(2,1)/cp, R(1,1)/cp);
            end
            eul = [roll, pitch, yaw]; % Return [roll, pitch, yaw]
        end
    end
    % ========================================================================

    methods (TestMethodSetup)
        % Setup for each test
        function setupTestObjects(testCase)
            % Create a clean figure and axes for graphics tests
            testCase.TestFig = figure('Visible', 'off');
            testCase.TestAx = axes('Parent', testCase.TestFig);

            % Initialize common test frames
            testCase.IdentityFrame = ReferenceFrame3d(); % Default identity

            % Use local helper for rotation matrix
            eul_angles = [pi/4, pi/6, pi/3]; % roll, pitch, yaw
            rotM = testCase.local_eul2rotm(eul_angles, 'ZYX');
            testCase.RotatedFrame = ReferenceFrame3d(rotM, [0 0 0]);

            transV = [1; 2; 3];
            testCase.TranslatedFrame = ReferenceFrame3d(eye(3), transV');

            generalT = [rotM, transV; 0 0 0 1];
            testCase.GeneralFrame = ReferenceFrame3d(generalT);

            % Ensure graphics handles are initially empty (or invalid if setup fails)
            testCase.IdentityFrame.clear();
            testCase.RotatedFrame.clear();
            testCase.TranslatedFrame.clear();
            testCase.GeneralFrame.clear();
        end
    end

    methods (TestMethodTeardown)
        % Teardown for each test
        function closeFigure(testCase)
            % Close the figure used for graphics tests
             if ~isempty(testCase.TestFig) && isvalid(testCase.TestFig)
                close(testCase.TestFig);
            end
            testCase.TestFig = [];
            testCase.TestAx = [];
        end
    end

    methods (Test, TestTags={'Constructor', 'Setup'})

        function testDefaultConstructor(testCase)
            % Test construction with no arguments (should be identity)
            frame = ReferenceFrame3d();
            testCase.verifyEqual(frame.T, eye(4), 'AbsTol', testCase.AbsTol, ...
                'Default constructor should create an identity transform.');
            testCase.verifyEmpty(frame.h_transform, 'Default constructor should not create graphics handles.');
        end

        function testConstructorWith4x4(testCase)
            % Test construction with a valid 4x4 homogeneous transform
            T_in = testCase.GeneralFrame.T;
            frame = ReferenceFrame3d(T_in);
            testCase.verifyEqual(frame.T, T_in, 'AbsTol', testCase.AbsTol, ...
                'Constructor with 4x4 matrix failed.');
        end

        function testConstructorWith3x3AndOrigin(testCase)
            % Test construction with a 3x3 DCM and origin vector
            R_in = testCase.RotatedFrame.R;
            t_in = testCase.TranslatedFrame.origin;
            T_expected = [R_in, t_in; 0 0 0 1];
            frame = ReferenceFrame3d(R_in, t_in'); % Origin is 1x3 row vector
            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.AbsTol, ...
                'Constructor with 3x3 matrix and origin failed.');
        end

        function testConstructorWithReferenceFrame(testCase)
            % Test construction by copying another ReferenceFrame3d object
            frame_orig = testCase.GeneralFrame;
            frame_copy = ReferenceFrame3d(frame_orig);
            % Should only copy the transform, not graphics handles
            testCase.verifyEqual(frame_copy.T, frame_orig.T, 'AbsTol', testCase.AbsTol, ...
                'Constructor with ReferenceFrame3d input failed to copy T matrix.');
            testCase.verifyEmpty(frame_copy.h_transform, ...
                'Constructor with ReferenceFrame3d input should not copy graphics handles.');
        end

        % --- Removed Toolbox-dependent Constructor Tests ---
        % testConstructorWithQuaternion
        % testConstructorWithSO3
        % testConstructorWithSE3

        function testConstructorInvalid4x4_Size(testCase)
            % Test construction with invalid 4x4 matrix (wrong size)
            T_invalid = rand(3,4);
            testCase.verifyError(@() ReferenceFrame3d(T_invalid), ...
                'MATLAB:validation:IncompatibleSize', ...
                'Constructor should error for non-4x4 matrix input.');
            T_invalid = rand(4,3);
            testCase.verifyError(@() ReferenceFrame3d(T_invalid), ...
                'MATLAB:validation:IncompatibleSize', ...
                'Constructor should error for non-4x4 matrix input.');
             T_invalid = rand(3,3); % Should be handled by 3x3 case
             testCase.verifyWarningFree(@() ReferenceFrame3d(T_invalid));
        end

         function testConstructorInvalid4x4_NotFinite(testCase)
            % Test construction with invalid 4x4 matrix (non-finite)
            T_nan = eye(4); T_nan(1,1) = NaN;
            testCase.verifyError(@() ReferenceFrame3d(T_nan), ...
                'MATLAB:validators:mustBeFinite',... % Error comes from validate_transform -> validateattributes
                'Constructor should error for non-finite 4x4 matrix.');
            T_inf = eye(4); T_inf(2,3) = Inf;
             testCase.verifyError(@() ReferenceFrame3d(T_inf), ...
                'MATLAB:validators:mustBeFinite',...
                'Constructor should error for non-finite 4x4 matrix.');
         end

        function testConstructorInvalid4x4_NotOrthogonal(testCase)
            % Test construction with invalid 4x4 matrix (non-orthogonal R)
            T_invalid = eye(4);
            T_invalid(1:3, 1:3) = rand(3,3); % Non-orthogonal
            testCase.verifyError(@() ReferenceFrame3d(T_invalid), ...
                'MATLAB:assertion:failed', ... % Error comes from validate_transform assert -> orthogonality check
                'Constructor should error for non-orthogonal rotation submatrix.');
        end

        function testConstructorInvalid4x4_WrongDeterminant(testCase)
            % Test construction with invalid 4x4 matrix (det(R) != 1)
            T_invalid = eye(4);
            % Use local helper to create a valid rotation first
            R_valid = testCase.local_eul2rotm([pi/4, pi/6, pi/3], 'ZYX');
            T_invalid(1:3, 1:3) = R_valid;
            % Force non-unit determinant while trying to maintain orthogonality approx
            T_invalid(1,1) = T_invalid(1,1) * 1.1; 
            T_invalid(2,2) = T_invalid(2,2) / 1.1; 
            
            % Re-orthogonalize using SVD, but force determinant slightly off 1
            [U, ~, V] = svd(T_invalid(1:3,1:3));
            R_det_not_1 = U * diag([1, 1, det(U*V')*1.001]) * V'; % Ensure det is not 1

            T_invalid(1:3, 1:3) = R_det_not_1;

            % Check the determinant is indeed not 1 (within tolerance)
            testCase.assumeGreaterThan(abs(det(R_det_not_1) - 1), testCase.AbsTol*10, ...
                'Test setup failed: Could not create matrix with det != 1 for testing.');

            testCase.verifyError(@() ReferenceFrame3d(T_invalid), ...
                'MATLAB:assertion:failed', ... % Error comes from validate_transform assert -> determinant check
                'Constructor should error for rotation submatrix with determinant not equal to 1.');

            % Also test determinant = -1 (reflection)
             T_reflect = eye(4);
             T_reflect(1,1) = -1;
             testCase.verifyError(@() ReferenceFrame3d(T_reflect), ...
                'MATLAB:assertion:failed', ...
                'Constructor should error for reflection matrix (det=-1).');
        end

        function testConstructorInvalid3x3(testCase)
            % Test construction with invalid 3x3 matrix (non-orthogonal)
            R_invalid = rand(3,3);
            testCase.verifyError(@() ReferenceFrame3d(R_invalid, [0 0 0]), ...
                'MATLAB:assertion:failed', ...
                'Constructor should error for non-orthogonal 3x3 matrix.');
        end

        function testConstructorInvalidOriginSize(testCase)
            % Test construction with invalid origin size
            R_valid = eye(3);
            origin_invalid = [1 2];
            testCase.verifyError(@() ReferenceFrame3d(R_valid, origin_invalid), ...
                 'MATLAB:validation:IncompatibleSize', ... % Error from arguments block
                'Constructor should error for origin not size 1x3.');
            origin_invalid = [1; 2; 3]; % Column vector
             testCase.verifyError(@() ReferenceFrame3d(R_valid, origin_invalid), ...
                 'MATLAB:validation:IncompatibleSize', ...
                'Constructor should error for origin not size 1x3.');
        end

         function testConstructorUnsupportedScalarType(testCase)
             % Test construction with unsupported scalar type
             unsupported_input = 'not a frame';
              testCase.verifyError(@() ReferenceFrame3d(unsupported_input), ...
                 'MATLAB:InputParser:ArgumentFailedValidation', ... % Error from arguments block validation
                 'Constructor should error for unsupported scalar input types.');
              unsupported_input = struct('a', 1);
               testCase.verifyError(@() ReferenceFrame3d(unsupported_input), ...
                 'MATLAB:InputParser:ArgumentFailedValidation', ...
                 'Constructor should error for unsupported scalar input types.');
               
             % Verify numeric scalar types still error as expected
             unsupported_numeric = 123.45;
              testCase.verifyError(@() ReferenceFrame3d(unsupported_numeric), ...
                  'MATLAB:InputParser:ArgumentFailedValidation', ... % Should fail class check in setup
                  'Constructor should error for unsupported numeric scalar types.');
         end

        function testSetupMethod(testCase)
            % Test the setup method (similar logic to constructor)
            frame = ReferenceFrame3d(); % Start with identity
            R_in = testCase.RotatedFrame.R;
            t_in = testCase.TranslatedFrame.origin;
            T_expected = [R_in, t_in; 0 0 0 1];

            % Setup with 3x3 and origin
            frame.setup(R_in, t_in');
            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.AbsTol, ...
                'Setup method with 3x3 matrix and origin failed.');

            % Setup with 4x4
            T_new = testCase.IdentityFrame.T;
            frame.setup(T_new);
             testCase.verifyEqual(frame.T, T_new, 'AbsTol', testCase.AbsTol, ...
                'Setup method with 4x4 matrix failed.');

            % Setup with another ReferenceFrame3d
            frame_orig = testCase.GeneralFrame;
            frame.setup(frame_orig);
             testCase.verifyEqual(frame.T, frame_orig.T, 'AbsTol', testCase.AbsTol, ...
                'Setup method with ReferenceFrame3d input failed.');

            % Test setup errors (reuse constructor error tests)
             T_invalid = eye(4); T_invalid(1:3, 1:3) = rand(3,3);
             testCase.verifyError(@() frame.setup(T_invalid), ...
                'MATLAB:assertion:failed', 'Setup should validate input.');
        end
    end

    methods (Test, TestTags={'StaticMethods', 'Factory'})

        function testFromPointNormalSimple(testCase)
            point = [1, 2, 3];
            normal = [0, 0, 1]; % Align with Z-axis
            frame = ReferenceFrame3d.from_point_normal(point, normal);

            % Verify origin
            testCase.verifyEqual(frame.origin, point', 'AbsTol', testCase.AbsTol, ...
                'from_point_normal failed to set origin.');
            % Verify Z-axis
            testCase.verifyEqual(frame.z, normal', 'AbsTol', testCase.AbsTol, ...
                'from_point_normal failed to align Z-axis with normal.');
            % Verify orthogonality and determinant
            testCase.verifyTrue(abs(det(frame.R) - 1) < testCase.AbsTol, ...
                 'from_point_normal result has det(R) != 1.');
             testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.AbsTol, ...
                 'from_point_normal result R is not orthogonal.');
        end

        function testFromPointNormalXAligned(testCase)
            point = [1, 1, 1];
            normal = [1, 0, 0]; % Normal aligned with X-axis
            frame = ReferenceFrame3d.from_point_normal(point, normal);

            testCase.verifyEqual(frame.origin, point', 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.z, normal', 'AbsTol', testCase.AbsTol, ...
                 'from_point_normal failed to align Z-axis for X-aligned normal.');
            % Check that X basis is orthogonal to Z (normal)
            testCase.verifyEqual(dot(frame.x, frame.z), 0, 'AbsTol', testCase.AbsTol);
            % Check that Y basis is orthogonal to Z (normal)
            testCase.verifyEqual(dot(frame.y, frame.z), 0, 'AbsTol', testCase.AbsTol);
             % Check orthogonality
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.AbsTol);
        end

         function testFromPointNormalYAligned(testCase)
            point = [-1, 0, 5];
            normal = [0, 1, 0]; % Normal aligned with Y-axis
            frame = ReferenceFrame3d.from_point_normal(point, normal);

            testCase.verifyEqual(frame.origin, point', 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.z, normal', 'AbsTol', testCase.AbsTol, ...
                 'from_point_normal failed to align Z-axis for Y-aligned normal.');
            testCase.verifyEqual(dot(frame.x, frame.z), 0, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(dot(frame.y, frame.z), 0, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.AbsTol);
         end

        function testFromPointNormalNonUnitNormal(testCase)
            point = [1, 2, 3];
            normal = [0, 0, 5]; % Non-unit normal
            frame = ReferenceFrame3d.from_point_normal(point, normal);
            expected_z = [0; 0; 1];

            testCase.verifyEqual(frame.origin, point', 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.z, expected_z, 'AbsTol', testCase.AbsTol, ...
                'from_point_normal failed to normalize the normal vector.');
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.AbsTol);
             testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.AbsTol);
        end

        function testFromPointNormalInvalidInput(testCase)
             point = [1, 2, 3];
             normal = [0, 0, 1];
             % Non-finite point
             point_nan = [NaN 1 1];
             testCase.verifyError(@() ReferenceFrame3d.from_point_normal(point_nan, normal), ...
                 'MATLAB:validators:mustBeFinite');
             % Non-finite normal
             normal_inf = [1 Inf 1];
              testCase.verifyError(@() ReferenceFrame3d.from_point_normal(point, normal_inf), ...
                 'MATLAB:validators:mustBeFinite');
             % Incorrect size point
              point_bad_size = [1 2];
              testCase.verifyError(@() ReferenceFrame3d.from_point_normal(point_bad_size, normal), ...
                 'MATLAB:validation:IncompatibleSize'); % from arguments block
              % Incorrect size normal
              normal_bad_size = [1; 2; 3; 4];
              testCase.verifyError(@() ReferenceFrame3d.from_point_normal(point, normal_bad_size), ...
                 'MATLAB:validation:IncompatibleSize'); % from arguments block

             % Zero normal vector (causes division by zero in norm)
             normal_zero = [0 0 0];
             % This will likely error inside the function due to division by zero during normalization
             testCase.verifyError(@() ReferenceFrame3d.from_point_normal(point, normal_zero), ...
                 '', ... % Don't assume specific error ID for internal div by zero, could be Warning + NaN
                 'from_point_normal with zero normal should error or produce NaN.'); 
             % Check for NaN if it doesn't error (depends on MATLAB version behavior with 0/0)
              try
                  frame_nan = ReferenceFrame3d.from_point_normal(point, normal_zero);
                  testCase.verifyTrue(any(isnan(frame_nan.T(:))), 'Zero normal resulted in non-NaN frame.');
              catch
                  % An error is also acceptable here
                  testCase.verifyTrue(true); 
              end
        end

        function testFromCoplanarVectorsOrthogonal(testCase)
            v1 = [1, 0, 0]; % x-axis
            v2 = [0, 1, 0]; % y-axis
            origin = [5, 6, 7];
            frame = ReferenceFrame3d.from_coplanar_vectors(v1, v2, origin);

            % Expected: x=v1, y=v2, z=cross(v1,v2) (all normalized)
            expected_x = [1; 0; 0];
            expected_y = [0; 1; 0];
            expected_z = [0; 0; 1];

            testCase.verifyEqual(frame.x, expected_x, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.y, expected_y, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.z, expected_z, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.origin, origin', 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.AbsTol);
        end

        function testFromCoplanarVectorsNonOrthogonal(testCase)
            v1 = [1, 1, 0]; % Vector in xy plane
            v2 = [0, 1, 1]; % Vector not orthogonal to v1
            origin = [1, 1, 1];
            frame = ReferenceFrame3d.from_coplanar_vectors(v1, v2, origin);

            expected_x_unnorm = v1(:);
            expected_z_unnorm = cross(v1(:), v2(:));
            expected_y_unnorm = cross(expected_z_unnorm, expected_x_unnorm);

            expected_x = expected_x_unnorm / norm(expected_x_unnorm);
            expected_y = expected_y_unnorm / norm(expected_y_unnorm);
            expected_z = expected_z_unnorm / norm(expected_z_unnorm);


            testCase.verifyEqual(frame.x, expected_x, 'AbsTol', testCase.AbsTol, ...
                'Coplanar vectors: X-basis incorrect.');
            testCase.verifyEqual(frame.y, expected_y, 'AbsTol', testCase.AbsTol, ...
                 'Coplanar vectors: Y-basis incorrect.');
            testCase.verifyEqual(frame.z, expected_z, 'AbsTol', testCase.AbsTol, ...
                 'Coplanar vectors: Z-basis incorrect.');

            testCase.verifyEqual(frame.origin, origin', 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.AbsTol, ...
                 'Coplanar vectors: R matrix not orthogonal.');
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.AbsTol, ...
                 'Coplanar vectors: det(R) not 1.');
        end

        function testFromCoplanarVectorsDefaultOrigin(testCase)
            v1 = [1, 0, 0];
            v2 = [0, 1, 0];
            frame = ReferenceFrame3d.from_coplanar_vectors(v1, v2); % No origin specified

            testCase.verifyEqual(frame.origin, [0; 0; 0], 'AbsTol', testCase.AbsTol, ...
                'Coplanar vectors: Default origin should be zero.');
        end

        function testFromCoplanarVectorsCollinear(testCase)
            v1 = [1, 2, 3];
            v2 = [-2, -4, -6]; % Collinear with v1
            origin = [1, 1, 1];

            % cross(v1, v2) will be zero vector. norm(zero) is 0. Division by zero in vecnorm.
            % This should lead to NaN values in the rotation matrix.
             try
                 frame = ReferenceFrame3d.from_coplanar_vectors(v1, v2, origin);
                 % If it doesn't error, check for NaNs which indicate division by zero issues
                 testCase.verifyTrue(any(isnan(frame.T(:))), ...
                     'Collinear vectors should result in NaN values due to division by zero.');
             catch ME
                 % Allow any error here as division by zero is expected.
                  testCase.verifyTrue(true, sprintf('Expected error or NaN for collinear vectors, got: %s', ME.message));
             end

             % Test with zero vector input
             v1_zero = [0 0 0];
             v2_valid = [1 0 0];
             try
                frame = ReferenceFrame3d.from_coplanar_vectors(v1_zero, v2_valid, origin);
                testCase.verifyTrue(any(isnan(frame.T(:))), ...
                     'Zero input vector (v1) should result in NaN values.');
             catch ME
                  testCase.verifyTrue(true, sprintf('Expected error or NaN for zero v1, got: %s', ME.message));
             end

             try
                frame = ReferenceFrame3d.from_coplanar_vectors(v2_valid, v1_zero, origin);
                testCase.verifyTrue(any(isnan(frame.T(:))), ...
                     'Zero input vector (v2) should result in NaN values.');
              catch ME
                  testCase.verifyTrue(true, sprintf('Expected error or NaN for zero v2, got: %s', ME.message));
             end
        end

        function testValidateTransformValid(testCase)
            % Test validate_transform with valid matrices
            testCase.verifyWarningFree(@() ReferenceFrame3d.validate_transform(eye(4)));
            testCase.verifyWarningFree(@() ReferenceFrame3d.validate_transform(testCase.GeneralFrame.T));

            % Test slightly perturbed but valid matrix
            T_slight_perturb = testCase.GeneralFrame.T;
            % Need re-orthogonalization if perturbed
            [U,~,V] = svd(T_slight_perturb(1:3,1:3));
            R_fixed = U*V';
            if det(R_fixed) < 0, V(:,end) = -V(:,end); R_fixed = U*V'; end % Ensure det=1
            T_fixed = T_slight_perturb;
            T_fixed(1:3,1:3) = R_fixed;
            testCase.verifyWarningFree(@() ReferenceFrame3d.validate_transform(T_fixed));
        end

        function testValidateTransformInvalid(testCase)
            % Reuse tests from constructor validation
            % Wrong size
            T_invalid_size = rand(3,4);
            testCase.verifyError(@() ReferenceFrame3d.validate_transform(T_invalid_size), ...
                 'MATLAB:validation:IncompatibleSize');
            % Non-finite
            T_nan = eye(4); T_nan(1,1) = NaN;
             testCase.verifyError(@() ReferenceFrame3d.validate_transform(T_nan), ...
                'MATLAB:validators:mustBeFinite');
            % Non-orthogonal
            T_nonorth = eye(4); T_nonorth(1:3, 1:3) = rand(3,3);
             testCase.verifyError(@() ReferenceFrame3d.validate_transform(T_nonorth), ...
                'MATLAB:assertion:failed'); % Orthogonality check
            % Wrong determinant
             T_reflect = eye(4); T_reflect(1,1) = -1;
              testCase.verifyError(@() ReferenceFrame3d.validate_transform(T_reflect), ...
                'MATLAB:assertion:failed'); % Determinant check
        end
    end

    methods (Test, TestTags={'Math', 'Utility', 'Transform'})

        function testLocal2BaseSingleVector(testCase)
            frame = testCase.GeneralFrame;
            local_pt = [1; 0; 0]; % Point along local x-axis

            % Expected result: Rotate [1;0;0] by R, then add t
            expected_base_pt = frame.R * local_pt + frame.origin;

            % Test Nx3 input format
            base_pt_Nx3 = frame.local2base(local_pt');
            testCase.verifyEqual(base_pt_Nx3', expected_base_pt, 'AbsTol', testCase.AbsTol, ...
                'local2base (Nx3 input) failed.');

            % Test separate x,y,z input format
            [bx, by, bz] = frame.local2base(local_pt(1), local_pt(2), local_pt(3));
            base_pt_xyz = [bx; by; bz];
             testCase.verifyEqual(base_pt_xyz, expected_base_pt, 'AbsTol', testCase.AbsTol, ...
                'local2base (x,y,z input) failed.');
        end

         function testLocal2BaseMultipleVectors(testCase)
            frame = testCase.GeneralFrame;
            local_pts = [1 0 0; 0 1 0; 0 0 1; 1 1 1]'; % 3x4 matrix (points as columns)

            % Expected result: R * local_pts + t (broadcast t)
            expected_base_pts = frame.R * local_pts + frame.origin;

            % Test Nx3 input format (Input should be 4x3)
            base_pts_Nx3 = frame.local2base(local_pts');
            testCase.verifyEqual(base_pts_Nx3', expected_base_pts, 'AbsTol', testCase.AbsTol, ...
                'local2base (Nx3 multiple points) failed.');

            % Test separate x,y,z input format (Input should be 4x1 vectors)
            [bx, by, bz] = frame.local2base(local_pts(1,:)', local_pts(2,:)', local_pts(3,:)');
            base_pts_xyz = [bx'; by'; bz']; % Needs reshaping back
             testCase.verifyEqual(base_pts_xyz, expected_base_pts, 'AbsTol', testCase.AbsTol, ...
                'local2base (x,y,z multiple points) failed.');
         end

         function testLocal2BaseIdentity(testCase)
             frame = testCase.IdentityFrame;
             local_pt = [10; -5; 2];

             base_pt = frame.local2base(local_pt');
             testCase.verifyEqual(base_pt', local_pt, 'AbsTol', testCase.AbsTol, ...
                 'local2base with identity transform failed.');
         end

         function testLocal2BasePureTranslation(testCase)
             frame = testCase.TranslatedFrame;
             local_pt = [1; 1; 1];
             expected_base_pt = local_pt + frame.origin;

             base_pt = frame.local2base(local_pt');
              testCase.verifyEqual(base_pt', expected_base_pt, 'AbsTol', testCase.AbsTol, ...
                 'local2base with pure translation failed.');
         end

         function testLocal2BasePureRotation(testCase)
             frame = testCase.RotatedFrame;
             local_pt = [0; 1; 0]; % Local y-axis
             expected_base_pt = frame.R * local_pt; % t is zero

             base_pt = frame.local2base(local_pt');
             testCase.verifyEqual(base_pt', expected_base_pt, 'AbsTol', testCase.AbsTol, ...
                 'local2base with pure rotation failed.');
         end

         function testLocal2BaseInvalidInputSize(testCase)
             frame = testCase.GeneralFrame;
             % Incorrect number of columns for NxM format
             bad_vec_col = [1 2; 3 4; 5 6]; % 3x2, expected Nx3
              testCase.verifyError(@() frame.local2base(bad_vec_col), ...
                  'MATLAB:ReferenceFrame3d:local2base', ... % Check error ID from function
                  'local2base should error if input matrix is not Nx3');
             % Incorrect dimension (needs 4th homogeneous coord internally)
             bad_vec_dim = [1 2 3 4 5]; % 1x5
               testCase.verifyError(@() frame.local2base(bad_vec_dim), ...
                   'MATLAB:ReferenceFrame3d:local2base', ... % Check error ID from function
                   'local2base should error if input matrix is not Nx3');
             % Incorrect number of arguments for x,y,z format
             testCase.verifyError(@() frame.local2base(1, 2), ...
                  'MATLAB:minrhs', ... % Standard MATLAB error for wrong number of inputs
                  'local2base should error if not 1 or 3 vector inputs provided');
             % Mismatched sizes for x,y,z format
             testCase.verifyError(@() frame.local2base([1;2], [3;4], [5;6;7]), ...
                  'MATLAB:matrix:DimensionsMismatch', ... % Error during concatenation
                  'local2base should error if x,y,z inputs have mismatched sizes');
         end

        function testBase2LocalSingleVector(testCase)
            frame = testCase.GeneralFrame;
            base_pt = frame.origin + frame.x; % Point one unit along local x from origin, expressed in base
            expected_local_pt = [1; 0; 0];

            % Test Nx3 input format
            local_pt_Nx3 = frame.base2local(base_pt');
            testCase.verifyEqual(local_pt_Nx3', expected_local_pt, 'AbsTol', testCase.AbsTol, ...
                'base2local (Nx3 input) failed.');

            % Test separate x,y,z input format
            [lx, ly, lz] = frame.base2local(base_pt(1), base_pt(2), base_pt(3));
            local_pt_xyz = [lx; ly; lz];
             testCase.verifyEqual(local_pt_xyz, expected_local_pt, 'AbsTol', testCase.AbsTol, ...
                'base2local (x,y,z input) failed.');
        end

        function testBase2LocalMultipleVectors(testCase)
             frame = testCase.GeneralFrame;
             % Points along axes in base frame, relative to frame's origin
             base_pts = [frame.origin + frame.x, frame.origin + frame.y, frame.origin + frame.z, frame.origin]; % 3x4
             expected_local_pts = [1 0 0 0; 0 1 0 0; 0 0 1 0]; % 3x4

             % Test Nx3 input format (Input should be 4x3)
             local_pts_Nx3 = frame.base2local(base_pts');
              testCase.verifyEqual(local_pts_Nx3', expected_local_pts, 'AbsTol', testCase.AbsTol, ...
                 'base2local (Nx3 multiple points) failed.');

             % Test separate x,y,z input format (Input should be 4x1 vectors)
             [lx, ly, lz] = frame.base2local(base_pts(1,:)', base_pts(2,:)', base_pts(3,:)');
             local_pts_xyz = [lx'; ly'; lz'];
              testCase.verifyEqual(local_pts_xyz, expected_local_pts, 'AbsTol', testCase.AbsTol, ...
                 'base2local (x,y,z multiple points) failed.');
        end

        function testBase2LocalIdentity(testCase)
             frame = testCase.IdentityFrame;
             base_pt = [10; -5; 2];

             local_pt = frame.base2local(base_pt');
             testCase.verifyEqual(local_pt', base_pt, 'AbsTol', testCase.AbsTol, ...
                 'base2local with identity transform failed.');
        end

        function testBase2LocalPureTranslation(testCase)
             frame = testCase.TranslatedFrame;
             base_pt = [5; 6; 7];
             expected_local_pt = base_pt - frame.origin;

             local_pt = frame.base2local(base_pt');
              testCase.verifyEqual(local_pt', expected_local_pt, 'AbsTol', testCase.AbsTol, ...
                 'base2local with pure translation failed.');
        end

        function testBase2LocalPureRotation(testCase)
             frame = testCase.RotatedFrame;
             base_pt = frame.y; % Local y-axis vector expressed in base frame
             expected_local_pt = [0; 1; 0];

             local_pt = frame.base2local(base_pt');
             testCase.verifyEqual(local_pt', expected_local_pt, 'AbsTol', testCase.AbsTol, ...
                 'base2local with pure rotation failed.');
        end

         function testBase2LocalInvalidInputSize(testCase)
             frame = testCase.GeneralFrame;
             % Incorrect number of columns for NxM format
             bad_vec_col = [1 2; 3 4; 5 6]; % 3x2, expected Nx3
              testCase.verifyError(@() frame.base2local(bad_vec_col), ...
                  'MATLAB:ReferenceFrame3d:base2local', ... % Check error ID from function
                  'base2local should error if input matrix is not Nx3');
              % Incorrect dimension
             bad_vec_dim = [1 2 3 4 5]; % 1x5
               testCase.verifyError(@() frame.base2local(bad_vec_dim), ...
                   'MATLAB:ReferenceFrame3d:base2local', ... % Check error ID from function
                   'base2local should error if input matrix is not Nx3');
             % Incorrect number of arguments for x,y,z format
             testCase.verifyError(@() frame.base2local(1, 2), ...
                  'MATLAB:minrhs', ...
                  'base2local should error if not 1 or 3 vector inputs provided');
             % Mismatched sizes for x,y,z format
             testCase.verifyError(@() frame.base2local([1;2], [3;4], [5;6;7]), ...
                  'MATLAB:matrix:DimensionsMismatch', ...
                  'base2local should error if x,y,z inputs have mismatched sizes');
         end

        function testLocalBaseInverseRelation(testCase)
            % Test that local2base(base2local(pt)) == pt
            frame = testCase.GeneralFrame;
            base_pt_orig = randn(5, 3); % Multiple points

            local_pt = frame.base2local(base_pt_orig);
            base_pt_reconv = frame.local2base(local_pt);

            testCase.verifyEqual(base_pt_reconv, base_pt_orig, 'AbsTol', testCase.AbsTol, ...
                'local2base(base2local(pt)) did not return original pt.');

            % Test that base2local(local2base(pt)) == pt
            local_pt_orig = randn(4, 3);
            base_pt = frame.local2base(local_pt_orig);
            local_pt_reconv = frame.base2local(base_pt);

            testCase.verifyEqual(local_pt_reconv, local_pt_orig, 'AbsTol', testCase.AbsTol, ...
                'base2local(local2base(pt)) did not return original pt.');
        end

        function testLocalBaseWithArrayInput(testCase)
            % Test local2base/base2local when 'this' is an array (uses compose)
            frame1 = testCase.RotatedFrame;
            frame2 = testCase.TranslatedFrame;
            frame_array = [frame1; frame2]; % Represents frame1 * frame2

            frame_composed = frame1 * frame2; % Explicit composition

            local_pt = [1; 1; 1];

            % Test local2base
            base_pt_array = frame_array.local2base(local_pt');
            base_pt_composed = frame_composed.local2base(local_pt');
            testCase.verifyEqual(base_pt_array, base_pt_composed, 'AbsTol', testCase.AbsTol, ...
                'local2base with frame array did not match composed frame.');

            % Test base2local
            base_pt = [5;5;5];
            local_pt_array = frame_array.base2local(base_pt');
            local_pt_composed = frame_composed.base2local(base_pt');
             testCase.verifyEqual(local_pt_array, local_pt_composed, 'AbsTol', testCase.AbsTol, ...
                'base2local with frame array did not match composed frame.');
        end

        function testTranslate(testCase)
            frame = testCase.GeneralFrame.copy(); % Work on a copy
            orig_T = frame.T;
            dxyz = [0.1; -0.2; 0.5];

            frame.translate(dxyz);

            expected_t = orig_T(1:3, 4) + dxyz;
            expected_R = orig_T(1:3, 1:3);

            testCase.verifyEqual(frame.R, expected_R, 'AbsTol', testCase.AbsTol, ...
                'Translate should not change the rotation matrix.');
            testCase.verifyEqual(frame.origin, expected_t, 'AbsTol', testCase.AbsTol, ...
                'Translate failed to update the origin correctly.');

            % Test invalid input size
            dxyz_bad = [1; 2];
             testCase.verifyError(@() frame.translate(dxyz_bad), ...
                 'MATLAB:validation:IncompatibleSize', ...
                 'Translate should error for input not size 3x1.');
             dxyz_bad = [1 2 3]; % Row vector
              testCase.verifyError(@() frame.translate(dxyz_bad), ...
                 'MATLAB:validation:IncompatibleSize', ...
                 'Translate should error for input not size 3x1.');
             % Test non-finite input
             dxyz_nan = [1; NaN; 3];
              testCase.verifyError(@() frame.translate(dxyz_nan), ...
                 'MATLAB:validators:mustBeFinite', ...
                 'Translate should error for non-finite input.');
        end

        function testReposition(testCase)
            frame = testCase.GeneralFrame.copy(); % Work on a copy
            orig_R = frame.R;
            new_pos = [-5; -6; -7];

            frame.reposition(new_pos);

            testCase.verifyEqual(frame.R, orig_R, 'AbsTol', testCase.AbsTol, ...
                'Reposition should not change the rotation matrix.');
            testCase.verifyEqual(frame.origin, new_pos, 'AbsTol', testCase.AbsTol, ...
                'Reposition failed to set the new origin.');

            % Test invalid input size
            new_pos_bad = [1; 2];
             testCase.verifyError(@() frame.reposition(new_pos_bad), ...
                 'MATLAB:validation:IncompatibleSize', ...
                 'Reposition should error for input not size 3x1.');
             new_pos_bad = [1 2 3]; % Row vector
              testCase.verifyError(@() frame.reposition(new_pos_bad), ...
                 'MATLAB:validation:IncompatibleSize', ...
                 'Reposition should error for input not size 3x1.');
            % Test non-finite input
             new_pos_nan = [1; NaN; 3];
              testCase.verifyError(@() frame.reposition(new_pos_nan), ...
                 'MATLAB:validators:mustBeFinite', ...
                 'Reposition should error for non-finite input.');
        end

        function testRotateDCM(testCase)
            frame = testCase.GeneralFrame.copy();
            orig_T = frame.T;

            % Rotation matrix for 90 degrees around Z
            dcm_rot = [0 -1 0; 1 0 0; 0 0 1];
            T_rot = eye(4);
            T_rot(1:3, 1:3) = dcm_rot;

            frame.rotate(dcm_rot);

            % Rotation is applied as T_new = T_old * T_rot
            expected_T = orig_T * T_rot;

            testCase.verifyEqual(frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                'Rotate with DCM produced incorrect transform.');

            % Test invalid DCM input (non-orthogonal)
             dcm_bad = rand(3,3);
              testCase.verifyError(@() frame.rotate(dcm_bad), ...
                  'MATLAB:assertion:failed', ... % Error comes from set.T validation
                  'Rotate should error if the resulting T matrix is invalid (due to bad DCM).');
              % Test invalid size input
              dcm_bad_size = rand(3,2);
              testCase.verifyError(@() frame.rotate(dcm_bad_size), ...
                   'MATLAB:validation:IncompatibleSize', ... % From arguments block
                   'Rotate should error if DCM is not 3x3.');
        end

        function testRotateEulerRad(testCase)
             frame = testCase.GeneralFrame.copy();
             orig_T = frame.T;

             % ZYX Euler angles (radians) [roll, pitch, yaw]
             roll = pi/2; pitch = 0; yaw = 0;

             % Equivalent DCM for post-multiplication using local helper
             T_rot = testCase.local_eul2tform([roll, pitch, yaw], 'ZYX');

             frame.rotate_euler(roll, pitch, yaw);

             % Calculate expected T_new = T_old * T_rot
             expected_T = orig_T * T_rot;

              testCase.verifyEqual(frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                 'Rotate with Euler angles (radians) failed.');

             % Test another sequence
             frame = testCase.GeneralFrame.copy(); % Reset
             orig_T = frame.T;
             roll = pi/6; pitch = -pi/4; yaw = pi/3;
             T_rot = testCase.local_eul2tform([roll, pitch, yaw], 'ZYX');
             frame.rotate_euler(roll, pitch, yaw);
             expected_T = orig_T * T_rot;
              testCase.verifyEqual(frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                 'Rotate with Euler angles (radians) failed for non-trivial angles.');
        end

        function testRotateEulerDeg(testCase)
            frame = testCase.GeneralFrame.copy();
            orig_T = frame.T;

            % ZYX Euler angles (degrees)
            roll_deg = 90; pitch_deg = 0; yaw_deg = 0;

            % Equivalent T_rot using radians and local helper
            roll_rad = roll_deg * pi/180;
            pitch_rad = pitch_deg * pi/180;
            yaw_rad = yaw_deg * pi/180;
            T_rot = testCase.local_eul2tform([roll_rad, pitch_rad, yaw_rad], 'ZYX');

            frame.rotate_eulerd(roll_deg, pitch_deg, yaw_deg);

            expected_T = orig_T * T_rot;

             testCase.verifyEqual(frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                'Rotate with Euler angles (degrees) failed.');

             % Test another sequence
             frame = testCase.GeneralFrame.copy(); % Reset
             orig_T = frame.T;
             roll_deg = 30; pitch_deg = -45; yaw_deg = 60;
             roll_rad = roll_deg * pi/180;
             pitch_rad = pitch_deg * pi/180;
             yaw_rad = yaw_deg * pi/180;
             T_rot = testCase.local_eul2tform([roll_rad, pitch_rad, yaw_rad], 'ZYX');
             frame.rotate_eulerd(roll_deg, pitch_deg, yaw_deg);
             expected_T = orig_T * T_rot;
              testCase.verifyEqual(frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                 'Rotate with Euler angles (degrees) failed for non-trivial angles.');
        end

        function testComposeTwoFrames(testCase)
            frame1 = testCase.RotatedFrame;
            frame2 = testCase.TranslatedFrame;

            composed_frame = compose(frame1, frame2); % T = T1 * T2
            expected_T = frame1.T * frame2.T;

            testCase.verifyClass(composed_frame, 'ReferenceFrame3d');
            testCase.verifyEqual(composed_frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                'Compose two frames failed.');
        end

        function testComposeMultipleFrames(testCase)
            frame1 = testCase.RotatedFrame;
            frame2 = testCase.TranslatedFrame;
            frame3 = testCase.GeneralFrame;

            % Compose function takes varargin, treated as vertical concatenation
            composed_frame = compose(frame1, frame2, frame3); % T = T1 * T2 * T3
            expected_T = frame1.T * frame2.T * frame3.T;

            testCase.verifyEqual(composed_frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                 'Compose multiple frames failed.');

            % Also test passing as a column vector
            frame_array = [frame1; frame2; frame3];
            composed_frame_array = compose(frame_array);
             testCase.verifyEqual(composed_frame_array.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                 'Compose multiple frames from array input failed.');
        end

        function testComposeIdentity(testCase)
             frame1 = testCase.GeneralFrame;
             frame_id = testCase.IdentityFrame;

             % Compose with identity first
             composed1 = compose(frame_id, frame1);
             testCase.verifyEqual(composed1.T, frame1.T, 'AbsTol', testCase.AbsTol, ...
                 'Compose with identity (Id * F) failed.');

             % Compose with identity last
             composed2 = compose(frame1, frame_id);
             testCase.verifyEqual(composed2.T, frame1.T, 'AbsTol', testCase.AbsTol, ...
                  'Compose with identity (F * Id) failed.');

             % Compose identity only
             composed3 = compose(frame_id);
             testCase.verifyEqual(composed3.T, frame_id.T, 'AbsTol', testCase.AbsTol, ...
                 'Compose with single identity frame failed.');

             % Compose multiple identities
             composed4 = compose(frame_id, frame_id, frame_id);
              testCase.verifyEqual(composed4.T, frame_id.T, 'AbsTol', testCase.AbsTol, ...
                 'Compose with multiple identity frames failed.');
        end

        function testComposeEmpty(testCase)
            % Test compose with empty array input
            empty_frame_array = ReferenceFrame3d.empty(0,1);
             testCase.verifyError(@() compose(empty_frame_array), 'MATLAB:badsubscript', ...
                 'Compose with empty array should error.');

             % Test compose with single frame input
             frame = testCase.GeneralFrame;
             composed_single = compose(frame);
             testCase.verifyEqual(composed_single.T, frame.T, 'AbsTol', testCase.AbsTol, ...
                 'Compose with single frame input failed.');
        end

        function testInv(testCase)
            frame = testCase.GeneralFrame;
            inv_frame = inv(frame); % Calculate inverse using the method

            % Calculate expected inverse manually
            R_inv_expected = frame.R';
            t_inv_expected = -R_inv_expected * frame.origin;
            T_inv_expected = [R_inv_expected, t_inv_expected; 0 0 0 1];

            testCase.verifyClass(inv_frame, 'ReferenceFrame3d');
            testCase.verifyEqual(inv_frame.T, T_inv_expected, 'AbsTol', testCase.AbsTol, ...
                'Inverse calculation failed.');
        end

         function testInvIdentity(testCase)
             frame = testCase.IdentityFrame;
             inv_frame = inv(frame);
             testCase.verifyEqual(inv_frame.T, frame.T, 'AbsTol', testCase.AbsTol, ...
                 'Inverse of identity should be identity.');
         end

        function testInvComposeRelation(testCase)
            % Test that inv(inv(F)) == F
            frame = testCase.GeneralFrame;
            inv_inv_frame = inv(inv(frame)); % Apply inverse twice
            testCase.verifyEqual(inv_inv_frame.T, frame.T, 'AbsTol', testCase.AbsTol, ...
                 'inv(inv(frame)) did not return the original frame.');

            % Test that F * inv(F) == Identity
            inv_frame = inv(frame);
            composed_identity = compose(frame, inv_frame);
            testCase.verifyEqual(composed_identity.T, eye(4), 'AbsTol', testCase.AbsTol, ...
                'frame * inv(frame) did not result in identity.');

            % Test that inv(F) * F == Identity
            composed_identity_rev = compose(inv_frame, frame);
            testCase.verifyEqual(composed_identity_rev.T, eye(4), 'AbsTol', testCase.AbsTol, ...
                'inv(frame) * frame did not result in identity.');
        end
    end

    methods (Test, TestTags={'Intersection'})

        function testIntersectPlaneSimple(testCase)
            % Frame at origin, xy plane, ray straight down from above
            frame = testCase.IdentityFrame;
            observer = [0 0 10];
            ray = [0 0 -1]; % Pointing straight down

            expected_p = [0 0 0];
            expected_dist = 10;

            [p, dist] = frame.intersect_plane(observer, ray, 'Slice', 'xy', 'Offset', 0, 'Debug', false);

            testCase.verifyEqual(p, expected_p, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(dist, expected_dist, 'AbsTol', testCase.AbsTol);
        end

        function testIntersectPlaneTranslatedRotated(testCase)
             % Frame rotated and translated, ray intersects its xy plane
             frame = testCase.GeneralFrame; % Use the general frame
             observer = frame.origin + frame.z * 10; % 10 units along frame's z-axis from origin
             ray = -frame.z; % Ray pointing along negative z-axis of the frame

             % Intersection should be at the frame's origin
             expected_p = frame.origin;
             expected_dist = 10; % Distance from observer to origin along ray

             [p, dist] = frame.intersect_plane(observer', ray', 'Slice', 'xy', 'Offset', 0, 'Debug', false);

             testCase.verifyEqual(p, expected_p', 'AbsTol', testCase.AbsTol); % p is 1x3
             testCase.verifyEqual(dist, expected_dist, 'AbsTol', testCase.AbsTol);
        end

        function testIntersectPlaneDifferentSlice(testCase)
            % Frame at origin, check xz plane intersection
            frame = testCase.IdentityFrame;
            observer = [0 10 0]; % Pointing from +y axis
            ray = [0 -1 0]; % Pointing towards origin along -y

            expected_p = [0 0 0];
            expected_dist = 10;

            [p, dist] = frame.intersect_plane(observer, ray, 'Slice', 'xz', 'Offset', 0, 'Debug', false);

            testCase.verifyEqual(p, expected_p, 'AbsTol', testCase.AbsTol, 'Slice xz failed.');

             % Check yz plane intersection
             observer = [10 0 0]; % Pointing from +x axis
             ray = [-1 0 0]; % Pointing towards origin along -x
             [p, dist] = frame.intersect_plane(observer, ray, 'Slice', 'yz', 'Offset', 0, 'Debug', false);
             testCase.verifyEqual(p, expected_p, 'AbsTol', testCase.AbsTol, 'Slice yz failed.');
        end

        function testIntersectPlaneOffset(testCase)
            % Frame at origin, xy plane offset along z
            frame = testCase.IdentityFrame;
            offset = 5;
            observer = [0 0 10];
            ray = [0 0 -1]; % Pointing straight down

            expected_p = [0 0 offset]; % Intersects at z = offset
            expected_dist = 10 - offset; % Distance from observer to plane

            [p, dist] = frame.intersect_plane(observer, ray, 'Slice', 'xy', 'Offset', offset, 'Debug', false);

            testCase.verifyEqual(p, expected_p, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(dist, expected_dist, 'AbsTol', testCase.AbsTol);

            % Test offset with rotated/translated frame
             frame_gen = testCase.GeneralFrame;
             offset_gen = -2;
             normal_gen = frame_gen.z; % Normal to xy plane
             expected_p_gen = frame_gen.origin + offset_gen * normal_gen; % Point on offset plane

             observer_gen = expected_p_gen + normal_gen * 10; % Observer 10 units away along normal
             ray_gen = -normal_gen; % Ray towards plane
             expected_dist_gen = 10;

             [p_gen, dist_gen] = frame_gen.intersect_plane(observer_gen', ray_gen', 'Slice', 'xy', 'Offset', offset_gen, 'Debug', false);

             testCase.verifyEqual(p_gen, expected_p_gen', 'AbsTol', testCase.AbsTol); % p is 1x3
             testCase.verifyEqual(dist_gen, expected_dist_gen, 'AbsTol', testCase.AbsTol);
        end

        function testIntersectPlaneParallelRay(testCase)
            % Ray parallel to the plane, should not intersect
            frame = testCase.IdentityFrame;
            observer = [0 0 10];
            ray = [1 0 0]; % Ray parallel to xy plane

            expected_p_nan = [NaN NaN NaN];

            [p, dist] = frame.intersect_plane(observer, ray, 'Slice', 'xy', 'Offset', 0, 'Debug', false);

            testCase.verifyEqual(p, expected_p_nan);
            % Distance should be Inf or NaN if denominator is exactly zero
            testCase.verifyTrue(isinf(dist) || isnan(dist));

            % Test with very small denominator (near parallel, check tolerance)
             observer_near = [0 0 testCase.AbsTol/10]; % Very close to plane
             ray_near = [1 0 0]; % Still parallel
             [p_near, dist_near] = frame.intersect_plane(observer_near, ray_near, 'Slice', 'xy', 'Offset', 0, 'Tol', testCase.AbsTol, 'Debug', false);
             testCase.verifyEqual(p_near, expected_p_nan, 'Near parallel ray should return NaN with tolerance.');
        end

        function testIntersectPlaneObserverOnPlane(testCase)
            % Observer starts exactly on the plane
            frame = testCase.IdentityFrame;
            observer = [1 2 0]; % On xy plane
            ray = [0 0 -1]; % Pointing away from plane

            expected_p = observer;
            expected_dist = 0;

            [p, dist] = frame.intersect_plane(observer, ray, 'Slice', 'xy', 'Offset', 0, 'Debug', false);

            testCase.verifyEqual(p, expected_p, 'AbsTol', testCase.AbsTol);
            testCase.verifyEqual(dist, expected_dist, 'AbsTol', testCase.AbsTol);

            % Ray along the plane (denominator is zero)
             ray_along = [1 0 0];
             expected_p_nan = [NaN NaN NaN];
             [p_along, dist_along] = frame.intersect_plane(observer, ray_along, 'Slice', 'xy', 'Offset', 0, 'Debug', false);
             testCase.verifyTrue(all(isnan(p_along)), 'Ray along plane did not return NaN position.');
             testCase.verifyTrue(isinf(dist_along) || isnan(dist_along), 'Ray along plane did not return Inf/NaN distance.');
        end

        function testIntersectPlaneDebugOption(testCase)
            % Just check that the Debug=true option runs without error
            frame = testCase.GeneralFrame;
            observer = [0 0 10];
            ray = [0 0 -1];

            testCase.verifyWarningFree(@() frame.intersect_plane(observer, ray, 'Debug', true));
            % Check if a figure was created (TestMethodTeardown will close it)
            testCase.verifyNotEmpty(findall(groot, 'Type', 'Figure'));
        end

        function testIntersectPlaneInvalidSlice(testCase)
             frame = testCase.IdentityFrame;
             observer = [0 0 10];
             ray = [0 0 -1];
             testCase.verifyError(@() frame.intersect_plane(observer, ray, 'Slice', 'xx'), ...
                 'MATLAB:mustBeMember'); % Error from validatestring
        end

    end

    methods (Test, TestTags={'NumericRepresentation', 'Conversion'})

        function testAsTransform(testCase)
            frame = testCase.GeneralFrame;
            T = frame.as_transform();
            testCase.verifyEqual(T, frame.T, 'AbsTol', testCase.AbsTol, ...
                'as_transform did not return the internal T matrix.');
            testCase.verifySize(T, [4 4]);
        end

        function testAsDCM(testCase)
            frame = testCase.GeneralFrame;
            R = frame.as_dcm();
            testCase.verifyEqual(R, frame.R, 'AbsTol', testCase.AbsTol, ...
                 'as_dcm did not return the internal R matrix.');
            testCase.verifySize(R, [3 3]);
        end

        function testAsEulerRad(testCase)
            % Create frame from known Euler angles (rad) and check conversion back
            eul_in = [pi/6, -pi/4, pi/3]; % [roll, pitch, yaw]
            R_in = testCase.local_eul2rotm(eul_in, 'ZYX');
            frame = ReferenceFrame3d(R_in);

            [roll_out, pitch_out, yaw_out] = frame.as_euler();
            eul_out = [roll_out, pitch_out, yaw_out];

            % Use local helper for comparison
            eul_expected = testCase.local_rotm2eul(R_in, 'ZYX'); 

            % Need to handle potential wrap-around issues (e.g., -pi vs pi)
            % Compare rotation matrices reconstructed from angles
            R_out = testCase.local_eul2rotm(eul_out, 'ZYX');
            testCase.verifyEqual(R_out, R_in, 'AbsTol', testCase.AbsTol, ...
                'Reconstructed R matrix from as_euler does not match original.');

            % Additionally check angle equivalence (handle wrap-around)
            angle_diff = abs(eul_out - eul_expected);
            angle_diff_wrapped = abs(angle_diff - 2*pi); % Check diff if wrapped around
            is_equivalent = all(angle_diff < testCase.AbsTol | angle_diff_wrapped < testCase.AbsTol);
            testCase.verifyTrue(is_equivalent, 'Euler angles (rad) mismatch after conversion.');

            % Test Gimbal Lock case (Pitch = +/- pi/2)
             eul_gimbal_in = [pi/3, pi/2, pi/6]; % Roll, pitch=90, yaw
             R_gimbal = testCase.local_eul2rotm(eul_gimbal_in, 'ZYX');
             frame_gimbal = ReferenceFrame3d(R_gimbal);
             [roll_g, pitch_g, yaw_g] = frame_gimbal.as_euler();
             eul_gimbal_out = [roll_g, pitch_g, yaw_g];

             % In gimbal lock, only the sum/difference of roll/yaw might be preserved
             % Check pitch and reconstructed rotation matrix
             R_out_g = testCase.local_eul2rotm(eul_gimbal_out, 'ZYX');
             testCase.verifyEqual(pitch_g, eul_gimbal_in(2), 'AbsTol', testCase.AbsTol, ...
                  'Euler pitch at gimbal lock incorrect.');
             testCase.verifyEqual(R_out_g, R_gimbal, 'AbsTol', testCase.AbsTol, ...
                  'Euler angles at gimbal lock do not reconstruct original rotation.');
        end

        function testAsEulerDeg(testCase)
            % Create frame from known Euler angles (deg) and check conversion back
            eul_in_deg = [30, -45, 60]; % [roll, pitch, yaw]
            eul_in_rad = eul_in_deg * pi/180;
            R_in = testCase.local_eul2rotm(eul_in_rad, 'ZYX');
            frame = ReferenceFrame3d(R_in);

            [roll_out_deg, pitch_out_deg, yaw_out_deg] = frame.as_eulerd();
            eul_out_deg = [roll_out_deg, pitch_out_deg, yaw_out_deg];

            % Compare reconstructed rotation matrices
            R_out = testCase.local_eul2rotm(eul_out_deg * pi/180, 'ZYX');
            testCase.verifyEqual(R_out, R_in, 'AbsTol', testCase.AbsTol, ...
                'Reconstructed R matrix from as_eulerd does not match original.');

             % Additionally check angle equivalence (handle wrap-around for 360 deg)
            eul_expected_deg = testCase.local_rotm2eul(R_in, 'ZYX') * 180/pi;
            angle_diff = abs(eul_out_deg - eul_expected_deg);
            angle_diff_wrapped = abs(angle_diff - 360); % Check diff if wrapped around
             % Check equivalence modulo 360
            is_equivalent = all(mod(angle_diff + testCase.AbsTol, 360) < 2*testCase.AbsTol | ...
                                mod(angle_diff_wrapped + testCase.AbsTol, 360) < 2*testCase.AbsTol );
            testCase.verifyTrue(is_equivalent, 'Euler angles (deg) mismatch after conversion.');
        end
    end

    % --- Removed Toolbox Interoperability Tests ---
    % methods (Test, TestTags={'Toolbox', 'Interoperability'})

    methods (Test, TestTags={'Overloads'})

        function testMTimes(testCase)
            % Test mtimes (*) overload for frame composition
            frame1 = testCase.RotatedFrame;
            frame2 = testCase.TranslatedFrame;

            composed_frame = frame1 * frame2; % Use overloaded operator

            % Expected result from direct multiplication
            T_expected = frame1.T * frame2.T;

            testCase.verifyClass(composed_frame, 'ReferenceFrame3d');
            testCase.verifyEqual(composed_frame.T, T_expected, 'AbsTol', testCase.AbsTol, ...
                'mtimes overload failed to compose transforms correctly.');

            % Test associativity: (A*B)*C == A*(B*C)
            frame3 = testCase.GeneralFrame;
            composed_ABC1 = (frame1 * frame2) * frame3;
            composed_ABC2 = frame1 * (frame2 * frame3);
             testCase.verifyEqual(composed_ABC1.T, composed_ABC2.T, 'AbsTol', testCase.AbsTol, ...
                'mtimes overload is not associative.');
        end

        function testTranspose(testCase)
            % Test transpose (.) overload (should transpose rotation part only)
            frame = testCase.GeneralFrame;
            orig_T = frame.T;

            transposed_frame = frame.'; % Apply transpose

            expected_R = orig_T(1:3, 1:3).';
            expected_t = orig_T(1:3, 4); % Translation should be unchanged
            expected_T = orig_T;
            expected_T(1:3, 1:3) = expected_R;

            testCase.verifyClass(transposed_frame, 'ReferenceFrame3d');
             testCase.verifyEqual(transposed_frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                 'Transpose (.) failed.');
             testCase.verifyEqual(transposed_frame.R, expected_R, 'AbsTol', testCase.AbsTol);
             testCase.verifyEqual(transposed_frame.origin, expected_t, 'AbsTol', testCase.AbsTol);

             % Test that transpose modifies the object in place
             frame_copy = frame.copy();
             frame_copy.'; % Modify in place
             testCase.verifyEqual(frame_copy.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                  'Transpose (.) did not modify object in place.');
        end

         function testCTranspose(testCase)
             % Test complex conjugate transpose (') overload (should be same as transpose)
             frame = testCase.GeneralFrame;
             orig_T = frame.T;

             ctransposed_frame = frame'; % Apply ctranspose

             expected_R = orig_T(1:3, 1:3)'; % Same as transpose for real matrices
             expected_t = orig_T(1:3, 4);
             expected_T = orig_T;
             expected_T(1:3, 1:3) = expected_R;

             testCase.verifyClass(ctransposed_frame, 'ReferenceFrame3d');
             testCase.verifyEqual(ctransposed_frame.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                  'ctranspose failed.');
             testCase.verifyEqual(ctransposed_frame.R, expected_R, 'AbsTol', testCase.AbsTol);
             testCase.verifyEqual(ctransposed_frame.origin, expected_t, 'AbsTol', testCase.AbsTol);

             % Test that ctranspose modifies the object in place
             frame_copy = frame.copy();
             frame_copy'; % Modify in place
             testCase.verifyEqual(frame_copy.T, expected_T, 'AbsTol', testCase.AbsTol, ...
                  'ctranspose did not modify object in place.');
         end

         function testTransposeIsInvRotation(testCase)
             % Transpose of rotation matrix is its inverse
             frame = testCase.RotatedFrame; % Pure rotation
             transposed_frame = frame.';
             inv_frame = inv(frame);

             % For pure rotation, transpose should equal inverse (within tolerance)
             testCase.verifyEqual(transposed_frame.T, inv_frame.T, 'AbsTol', testCase.AbsTol, ...
                 'Transpose of pure rotation frame did not equal its inverse.');
                 
             % For general frame, transpose is NOT the inverse
             frame_gen = testCase.GeneralFrame;
             transposed_gen = frame_gen.';
             inv_gen = inv(frame_gen);
             testCase.verifyNotEqual(transposed_gen.T, inv_gen.T, ...
                 'Transpose of general frame should NOT equal its inverse.');
             testCase.verifyEqual(transposed_gen.R, inv_gen.R, ...
                  'Transpose R should equal inverse R.');
             testCase.verifyNotEqual(transposed_gen.origin, inv_gen.origin, ...
                  'Transpose t should NOT equal inverse t.');
         end
    end

    methods (Test, TestTags={'Graphics', 'Plotting'})

        function testPlotMethod(testCase)
            % Test plot method - check graphics object creation and parenting
            frame = testCase.GeneralFrame;

            % Plot into the test axes created in setup
            frame.plot('Parent', testCase.TestAx);

            % Verify hgtransform handle exists and is valid
            testCase.verifyNotEmpty(frame.h_transform);
            testCase.verifyClass(frame.h_transform, 'matlab.graphics.primitive.Transform');
            testCase.verifyTrue(isvalid(frame.h_transform));

             % Verify plot group handle exists and is valid
             testCase.verifyNotEmpty(frame.h_plot_group);
             testCase.verifyClass(frame.h_plot_group, 'matlab.graphics.primitive.Group');
             testCase.verifyTrue(isvalid(frame.h_plot_group));

            % Verify parenting
            testCase.verifyEqual(frame.h_transform.Parent, testCase.TestAx);
            testCase.verifyEqual(frame.h_plot_group.Parent, frame.h_transform);

            % Verify matrix of transform matches frame's T
            testCase.verifyEqual(frame.h_transform.Matrix, frame.T, 'AbsTol', testCase.AbsTol);
        end

        function testPlotMultipleFrames(testCase)
            frame1 = testCase.IdentityFrame;
            frame2 = testCase.TranslatedFrame;
            frame_array = [frame1; frame2];

            frame_array.plot('Parent', testCase.TestAx);

            % Check handles for both frames
            testCase.verifyNotEmpty(frame1.h_transform); testCase.verifyTrue(isvalid(frame1.h_transform));
            testCase.verifyNotEmpty(frame2.h_transform); testCase.verifyTrue(isvalid(frame2.h_transform));
            testCase.verifyNotEmpty(frame1.h_plot_group); testCase.verifyTrue(isvalid(frame1.h_plot_group));
            testCase.verifyNotEmpty(frame2.h_plot_group); testCase.verifyTrue(isvalid(frame2.h_plot_group));

            % Check parenting (frame2 transform should be child of frame1 transform)
            testCase.verifyEqual(frame1.h_transform.Parent, testCase.TestAx);
            testCase.verifyEqual(frame2.h_transform.Parent, frame1.h_transform);
        end

        function testDrawPlaneMethod(testCase)
            % Test draw_plane method
            frame = testCase.GeneralFrame;

            % Need to plot the frame first to create h_transform
            frame.plot('Parent', testCase.TestAx);

            % Draw default xy plane
            h_plane = frame.draw_plane();

            testCase.verifyClass(h_plane, 'matlab.graphics.primitive.Surface');
            testCase.verifyTrue(isvalid(h_plane));
            testCase.verifyEqual(h_plane.Parent, frame.h_transform, ...
                'Plane parent should be the frame''s hgtransform.');
            testCase.verifyEqual(h_plane.Tag, 'XY_PLANE');

            % Test drawing other slices without error
             testCase.verifyWarningFree(@() frame.draw_plane('Slice', 'xz'));
             testCase.verifyWarningFree(@() frame.draw_plane('Slice', 'yz'));
             testCase.verifyWarningFree(@() frame.draw_plane('Slice', 'zy', 'Offset', 1, 'Size', [2 2]));

             % Check handle tag for other slice
             h_zy = frame.draw_plane('Slice', 'zy');
             testCase.verifyEqual(h_zy.Tag, 'ZY_PLANE');

             % Test invalid slice error
             testCase.verifyError(@() frame.draw_plane('Slice', 'ab'), 'MATLAB:unrecognizedStringChoice');
        end

        function testHgtransformMethod(testCase)
            % Test hgtransform handle retrieval and creation
            frame = testCase.GeneralFrame;

            % Get handle without parent - should use gca (test axes implicitly via setup)
            hgt = frame.hgtransform(); 
            testCase.verifyEqual(hgt.Parent, testCase.TestAx); % Check parent is the test axis
            testCase.verifyEqual(hgt, frame.h_transform); % Handle should be stored
            testCase.verifyEqual(hgt.Matrix, frame.T);

             % Get handle again - should return existing handle
             hgt_again = frame.hgtransform();
             testCase.verifyEqual(hgt_again, hgt);

             % Test parenting to another frame's transform
             frame1 = testCase.IdentityFrame;
             frame2 = testCase.TranslatedFrame;
             hgt1 = frame1.hgtransform(testCase.TestAx);
             hgt2 = frame2.hgtransform(frame1); % Parent to frame1's transform

             testCase.verifyEqual(hgt2.Parent, hgt1);
             testCase.verifyEqual(hgt2, frame2.h_transform);

             % Test re-parenting (should delete old graphics and create new)
             old_hgt = frame.h_transform; % This is hgt from above
             testCase.verifyTrue(isvalid(old_hgt));
             new_fig = figure('Visible','off'); % Create a new figure/axes environment
             testCase.addTeardown(@() close(new_fig)); % Ensure cleanup
             new_ax = axes('Parent',new_fig);
             new_hgt = frame.hgtransform(new_ax); % Reparent

             testCase.verifyNotEqual(new_hgt, old_hgt, 'Reparenting should create a new handle.'); 
             testCase.verifyFalse(isvalid(old_hgt), 'Old handle should be deleted on reparenting.'); 
             testCase.verifyTrue(isvalid(new_hgt), 'New handle should be valid after reparenting.');
             testCase.verifyEqual(new_hgt.Parent, new_ax, 'New handle has incorrect parent.');
        end

        function testUpdateHgtransform(testCase)
            % Test updating the graphics transform matrix after object changes
            frame = testCase.GeneralFrame;
            frame.plot('Parent', testCase.TestAx); % Create graphics first
            hgt = frame.h_transform;

            % Modify the frame (this should trigger update via set.T)
            dxyz = [1;1;1];
            frame.translate(dxyz);

            % Verify the transform matrix has updated automatically via set.T
             testCase.verifyEqual(hgt.Matrix, frame.T, 'AbsTol', testCase.AbsTol, ...
                'set.T should have updated the hgtransform matrix.');

            % Manually corrupt the matrix and call update
            hgt.Matrix = eye(4); % Make it different
            frame.update_hgtransform(); % Call update explicitly
            testCase.verifyEqual(hgt.Matrix, frame.T, 'AbsTol', testCase.AbsTol, ...
                'update_hgtransform failed to update matrix.');

             % Test update on array
             frame1 = testCase.IdentityFrame;
             frame2 = testCase.TranslatedFrame;
             frame_array = [frame1; frame2];
             frame_array.plot('Parent', testCase.TestAx);
             hgt1 = frame1.h_transform;
             hgt2 = frame2.h_transform;

             frame1.translate([1 0 0]);
             frame2.translate([0 1 0]);

             % Check they updated (implicitly via set.T)
             testCase.verifyEqual(hgt1.Matrix, frame1.T, 'AbsTol', testCase.AbsTol);
             testCase.verifyEqual(hgt2.Matrix, frame2.T, 'AbsTol', testCase.AbsTol);

             % Corrupt and update explicitly
             hgt1.Matrix = eye(4);
             hgt2.Matrix = eye(4);
             frame_array.update_hgtransform();
             testCase.verifyEqual(hgt1.Matrix, frame1.T, 'AbsTol', testCase.AbsTol);
             testCase.verifyEqual(hgt2.Matrix, frame2.T, 'AbsTol', testCase.AbsTol);
        end

        function testClearMethod(testCase)
            % Test clearing graphics handles
            frame = testCase.GeneralFrame;
            frame.plot('Parent', testCase.TestAx);
            frame.draw_plane(); % Add more graphics

            hgt = frame.h_transform;
            hpg = frame.h_plot_group;
            h_plane = findobj(hgt, 'Type', 'Surface'); % Get plane handle

            testCase.verifyTrue(isvalid(hgt));
            testCase.verifyTrue(isvalid(hpg));
            testCase.verifyTrue(all(isvalid(h_plane)));

            frame.clear();

            testCase.verifyFalse(isvalid(hgt), 'h_transform should be invalid after clear.');
            testCase.verifyFalse(isvalid(hpg), 'h_plot_group should be invalid after clear.');
             testCase.verifyFalse(any(isvalid(h_plane)), 'Plane handles should be invalid after clear.');
            % Properties holding handles should become empty
            testCase.verifyEmpty(frame.h_transform);
            testCase.verifyEmpty(frame.h_plot_group);

            % Test clear on array
            frame1 = testCase.IdentityFrame;
            frame2 = testCase.TranslatedFrame;
            frame_array = [frame1; frame2];
            frame_array.plot('Parent', testCase.TestAx);
            hgt1 = frame1.h_transform;
            hgt2 = frame2.h_transform;
            frame_array.clear();
            testCase.verifyFalse(isvalid(hgt1));
            testCase.verifyFalse(isvalid(hgt2));
            testCase.verifyEmpty(frame1.h_transform);
            testCase.verifyEmpty(frame2.h_transform);
        end

    end

    methods (Test, TestTags={'Properties', 'Accessors'})

        function testSetTUpdatesGraphics(testCase)
            % Test that setting T updates the graphics transform matrix
             frame = testCase.GeneralFrame.copy();
             frame.plot('Parent', testCase.TestAx); % Create graphics
             hgt = frame.h_transform;

             T_new = testCase.TranslatedFrame.T;
             frame.T = T_new; % Set T directly

             testCase.verifyEqual(hgt.Matrix, T_new, 'AbsTol', testCase.AbsTol, ...
                 'Setting T did not update the hgtransform matrix.');

             % Test case where graphics don't exist yet
             frame_no_graphics = testCase.GeneralFrame.copy();
             T_new2 = testCase.RotatedFrame.T;
              testCase.verifyWarningFree(@() set(frame_no_graphics, 'T', T_new2));
              testCase.verifyEmpty(frame_no_graphics.h_transform, ...
                  'Setting T should not create graphics if they don''t exist.');
        end

        function testGetDependentProperties(testCase)
            % Test the get accessors for dependent properties R, x, y, z, t
            frame = testCase.GeneralFrame;
            T = frame.T;

            % Get R
            R = frame.R;
            testCase.verifyEqual(R, T(1:3, 1:3), 'AbsTol', testCase.AbsTol, 'get.R failed.');
            testCase.verifySize(R, [3 3]);

            % Get x
            x = frame.x;
            testCase.verifyEqual(x, T(1:3, 1), 'AbsTol', testCase.AbsTol, 'get.x failed.');
            testCase.verifySize(x, [3 1]);

            % Get y
            y = frame.y;
            testCase.verifyEqual(y, T(1:3, 2), 'AbsTol', testCase.AbsTol, 'get.y failed.');
            testCase.verifySize(y, [3 1]);

            % Get z
            z = frame.z;
            testCase.verifyEqual(z, T(1:3, 3), 'AbsTol', testCase.AbsTol, 'get.z failed.');
            testCase.verifySize(z, [3 1]);

            % Get t
            t = frame.origin;
            testCase.verifyEqual(t, T(1:3, 4), 'AbsTol', testCase.AbsTol, 'get.origin failed.');
            testCase.verifySize(t, [3 1]);
        end

    end

    methods(Test, TestTags={'Mixin', 'Copyable'})

        function testCopyMethodShallowVsDeep(testCase)
            % Test the copy method from matlab.mixin.Copyable
            frame_orig = testCase.GeneralFrame;
            frame_orig.plot('Parent', testCase.TestAx); % Give it some graphics

            frame_copy = copy(frame_orig);

            % 1. Verify it's a different object handle
            testCase.verifyNotSameHandle(frame_orig, frame_copy);

            % 2. Verify T matrix is equal (value copy)
            testCase.verifyEqual(frame_copy.T, frame_orig.T, 'AbsTol', testCase.AbsTol);

            % 3. Modify original's T and check copy is unchanged
            orig_T_before_mod = frame_orig.T;
            frame_orig.translate([1 1 1]);
            testCase.verifyNotEqual(frame_copy.T, frame_orig.T, ...
                'Copy modification check failed: Modifying original T affected copy T.');
            testCase.verifyEqual(frame_copy.T, orig_T_before_mod, ...
                 'Copy modification check failed: Modifying original T affected copy T (value check).');


            % 4. Reset original T, modify copy's T and check original is unchanged
            frame_orig.T = orig_T_before_mod; % Reset original
            copy_T_before_mod = frame_copy.T;
            frame_copy.translate([2 2 2]);
            testCase.verifyNotEqual(frame_orig.T, frame_copy.T, ...
                 'Copy modification check failed: Modifying copy T affected original T.');
            testCase.verifyEqual(frame_orig.T, copy_T_before_mod, ...
                 'Copy modification check failed: Modifying copy T affected original T (value check).');


            % 5. Verify graphics handles are copied and independent
             testCase.verifyNotEmpty(frame_orig.h_transform, 'Original graphics handle missing.');
             testCase.verifyTrue(isvalid(frame_orig.h_transform));
             testCase.verifyNotEmpty(frame_copy.h_transform, 'Copied graphics handle missing.');
             testCase.verifyTrue(isvalid(frame_copy.h_transform));
             testCase.verifyNotEqual(frame_orig.h_transform, frame_copy.h_transform, ...
                 'Copied graphics handle should be different from original.');
             testCase.verifyEqual(frame_orig.h_transform.Parent, frame_copy.h_transform.Parent, ...
                 'Copied graphics should have the same parent.'); % Should be TestAx

            % 6. Modify original graphics (e.g., delete) and check copy's graphics
             orig_hgt = frame_orig.h_transform;
             copy_hgt = frame_copy.h_transform;
             delete(orig_hgt);
             testCase.verifyFalse(isvalid(orig_hgt));
             testCase.verifyTrue(isvalid(copy_hgt), 'Deleting original graphics affected copy.');

             % 7. Test copy of object without graphics
             frame_no_graphics = testCase.RotatedFrame; % Has no graphics yet
             copy_no_graphics = copy(frame_no_graphics);
             testCase.verifyEmpty(copy_no_graphics.h_transform);
             testCase.verifyEqual(copy_no_graphics.T, frame_no_graphics.T);
        end

    end

end