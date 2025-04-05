classdef test_ReferenceFrame3d < matlab.unittest.TestCase

    properties (Constant)
        Tol = 1e-10 % Tolerance for floating-point comparisons
    end

    methods (Test)

        %% Constructor and Update Tests
        function testConstructorDefault(testCase)
            frame = ReferenceFrame3d();
            testCase.verifyEqual(frame.T, eye(4), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.R, eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.origin, [0; 0; 0], 'AbsTol', testCase.Tol);
        end

        function testConstructorTransform(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_arb = [R_arb, origin_arb; 0 0 0 1];
            frame = ReferenceFrame3d(T_arb);
            testCase.verifyEqual(frame.T, T_arb, 'AbsTol', testCase.Tol);
        end

        function testConstructorDcmOrigin(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_expected = [R_arb, origin_arb; 0 0 0 1];
            frame = ReferenceFrame3d(R_arb, origin_arb'); % Constructor expects row vector origin
            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
        end

        function testConstructorFromFrame(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_arb = [R_arb, origin_arb; 0 0 0 1];
            frame1 = ReferenceFrame3d(T_arb);
            frame2 = ReferenceFrame3d(frame1); % Should copy T, not handle

            testCase.verifyEqual(frame2.T, frame1.T, 'AbsTol', testCase.Tol);
            testCase.verifyNotSameHandle(frame1, frame2);
        end

        function testConstructorDcmOnly(testCase)
            % Test constructor with only a 3x3 DCM, origin should default to zero.
            [R_arb, ~] = testCase.createArbitraryRotOrigin();
            T_expected = [R_arb, [0;0;0]; 0 0 0 1];
            frame = ReferenceFrame3d(R_arb); % Origin argument omitted
            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.origin, [0;0;0], 'AbsTol', testCase.Tol);
        end

        function testUpdateMethod(testCase)
            frame = ReferenceFrame3d(); % Start with identity
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_arb = [R_arb, origin_arb; 0 0 0 1];

            % Update using R and origin
            frame.update(R_arb, origin_arb'); % Update expects row vector origin
            testCase.verifyEqual(frame.T, T_arb, 'AbsTol', testCase.Tol);

            % Update using T
            frame.update(eye(4));
            testCase.verifyEqual(frame.T, eye(4), 'AbsTol', testCase.Tol);

            % Update using another frame
            frame_arb = ReferenceFrame3d(T_arb);
            frame.update(frame_arb);
            testCase.verifyEqual(frame.T, T_arb, 'AbsTol', testCase.Tol);
        end

        function testStaticFromEuler(testCase)
            angles_deg = [30, 45, 60]; % ZYX
            origin_row = [1, 2, 3];
            frame = ReferenceFrame3d.from_euler(angles_deg, origin_row, 'Sequence', 'zyx', 'Units', 'deg');

            R_expected = testCase.eul2rotm_local(angles_deg, 'zyx', 'deg');
            T_expected = [R_expected, origin_row'; 0 0 0 1];

            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.origin, origin_row', 'AbsTol', testCase.Tol); % Check origin separately
        end

        function testStaticFromPointNormal(testCase)
            point = [1 2 3];
            normal = [0 1 0]; % Simple normal along y-axis
            frame = ReferenceFrame3d.from_point_normal(point, normal);

            testCase.verifyEqual(frame.origin, point', 'AbsTol', testCase.Tol);
            % Normal should align with frame's z-axis
            testCase.verifyEqual(frame.z, normal'/norm(normal), 'AbsTol', testCase.Tol);
            % Verify R is orthonormal (implicitly checked by set.T, but good practice)
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.Tol);
            % Verify x, y, z form right-handed system
            testCase.verifyEqual(cross(frame.x, frame.y), frame.z, 'AbsTol', testCase.Tol);
        end

        function testUpdateWithFrame(testCase)
            % Test update using another ReferenceFrame3d object.
            frameToUpdate = ReferenceFrame3d(); % Start with identity
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_arb = [R_arb, origin_arb; 0 0 0 1];
            sourceFrame = ReferenceFrame3d(T_arb);

            frameToUpdate.update(sourceFrame); % Update using the frame object

            testCase.verifyEqual(frameToUpdate.T, T_arb, 'AbsTol', testCase.Tol);
            testCase.verifyNotSameHandle(frameToUpdate, sourceFrame);
        end

        function testUpdateDcmOnly(testCase)
            % Test update with only a 3x3 DCM, origin should default to zero.
            frame = ReferenceFrame3d.from_euler([10 20 30],[1 2 3]); % Start non-identity
            [R_arb, ~] = testCase.createArbitraryRotOrigin();
            T_expected = [R_arb, [0;0;0]; 0 0 0 1]; % Expect origin to be zeroed

            frame.update(R_arb); % Origin argument omitted

            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.origin, [0;0;0], 'AbsTol', testCase.Tol);
        end

        function testUpdateUnsupportedType(testCase)
            % Test update with an unsupported scalar type.
            frame = ReferenceFrame3d();
            badInput = struct('a', 1); % An unsupported scalar type
            % Check for the specific error from the 'otherwise' block
            % Make sure your error call has an ID: error('ReferenceFrame3d:update:UnsupportedType', ...)
            testCase.verifyError(@() frame.update(badInput), ?MException, ...
                'Update should error for unsupported scalar types.');
        end

        function testUpdateQuaternion(testCase)
            % Test update using a quaternion object (conditional via try-catch).
            try
                frame = ReferenceFrame3d(); % Start identity
                R_arb = testCase.eul2rotm_local([15, -25, 35], 'zyx', 'deg');
                origin_arb = [4; 5; 6];
                T_expected = [R_arb, origin_arb; 0 0 0 1];
                quat_arb = quaternion(R_arb, 'rotmat', 'frame'); % This line might error

                frame.update(quat_arb, origin_arb'); % Update with quaternion and origin

                testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);

                % Test update with quaternion only (origin should be zero)
                T_expected_R_only = [R_arb, [0;0;0]; 0 0 0 1];
                frame.update(quat_arb); % Update with quaternion only
                testCase.verifyEqual(frame.T, T_expected_R_only, 'AbsTol', testCase.Tol);

            catch me
                if strcmp(me.identifier, 'MATLAB:UndefinedFunction') && contains(me.message, 'quaternion')
                    % user doesn't have the right toolbox; skip test gracefully
                    testCase.log(1, 'Skipping quaternion update test - quaternion function not found.');
                    return
                else
                    % Different error occurred, fail the test
                    rethrow(me);
                end
            end
        end

        function testUpdateSE3(testCase)
            % Test update using an se3 object (conditional via try-catch).
            try
                frame = ReferenceFrame3d(); % Start identity
                R_arb = testCase.eul2rotm_local([-10, 5, -15], 'zyx', 'deg');
                origin_arb = [7; 8; 9];
                T_expected = [R_arb, origin_arb; 0 0 0 1];
                se3_arb = se3(T_expected); % This line might error

                frame.update(se3_arb); % Update with se3 object

                testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
                % Note: The 'origin' argument in update is ignored when input is se3

            catch me
                 if strcmp(me.identifier, 'MATLAB:UndefinedFunction') && contains(me.message, 'se3')
                    % user doesn't have the right toolbox; skip test gracefully
                    testCase.log(1, 'Skipping SE3 update test - se3 function not found.');
                    return
                else
                    % Different error occurred, fail the test
                    rethrow(me);
                end
            end
        end

        function testUpdateSO3(testCase)
             % Test update using an so3 object (conditional via try-catch).
             try
                frame = ReferenceFrame3d(); % Start identity
                R_arb = testCase.eul2rotm_local([22, 33, 44], 'zyx', 'deg');
                origin_arb = [1; 1; 2];
                T_expected = [R_arb, origin_arb; 0 0 0 1];
                so3_arb = so3(R_arb); % This line might error

                frame.update(so3_arb, origin_arb'); % Update with so3 and origin

                testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);

                % Test update with so3 only (origin should be zero)
                T_expected_R_only = [R_arb, [0;0;0]; 0 0 0 1];
                frame.update(so3_arb); % Update with so3 only
                testCase.verifyEqual(frame.T, T_expected_R_only, 'AbsTol', testCase.Tol);

             catch me
                 if strcmp(me.identifier, 'MATLAB:UndefinedFunction') && contains(me.message, 'so3')
                    % user doesn't have the right toolbox; skip test gracefully
                    testCase.log(1, 'Skipping SO3 update test - so3 function not found.');
                    return
                else
                    % Different error occurred, fail the test
                    rethrow(me);
                end
             end
        end

        function testStaticFromPointNormal_YAxis(testCase)
            % Test from_point_normal with y-axis normal
            point = [1 2 3];
            normal = [0 1 0]; % Simple normal along y-axis
            frame = ReferenceFrame3d.from_point_normal(point, normal);

            testCase.verifyEqual(frame.origin, point', 'AbsTol', testCase.Tol);
            % Normal should align with frame's z-axis
            testCase.verifyEqual(frame.z, [0;1;0], 'AbsTol', testCase.Tol);
            % Check basis vectors form a valid rotation matrix
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.Tol);
            % Check handedness
            testCase.verifyEqual(cross(frame.x, frame.y), frame.z, 'AbsTol', testCase.Tol);
        end

        function testStaticFromPointNormal_XAxisEdge(testCase)
            % Test from_point_normal edge case where normal is close to x-axis
            point = [-1 0 5];
            % Normal slightly off the x-axis, forcing the 'else' branch for 'whatever'
            normal = [0.99 0.1 0.05];
            normal = normal / norm(normal); % Normalize before passing
            frame = ReferenceFrame3d.from_point_normal(point, normal);

            testCase.verifyEqual(frame.origin, point', 'AbsTol', testCase.Tol);
            % Normal should align with frame's z-axis
            testCase.verifyEqual(frame.z, normal', 'AbsTol', testCase.Tol);
            % Check basis vectors form a valid rotation matrix
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.Tol);
             % Check handedness
            testCase.verifyEqual(cross(frame.x, frame.y), frame.z, 'AbsTol', testCase.Tol);
        end

        function testStaticFromCoplanarVectors(testCase)
            % Test from_coplanar_vectors with non-unit, non-orthogonal vectors
            v1 = [1 1 0]; % Will become X basis (normalized)
            v2 = [0 2 0]; % Used to find Z = cross(v1,v2), then Y = cross(Z,X)
            origin = [1 2 3];
            frame = ReferenceFrame3d.from_coplanar_vectors(v1, v2, origin);

            x_expected = v1' / norm(v1);
            z_expected = cross(v1, v2)';
            z_expected = z_expected / norm(z_expected);
            y_expected = cross(z_expected, x_expected); % Should already be unit

            testCase.verifyEqual(frame.origin, origin', 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.x, x_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.y, y_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.z, z_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.Tol);
        end

        function testStaticFromEulerUnitsRad(testCase)
            % Test from_euler using radians
            angles_rad = [pi/6, pi/4, pi/3]; % ZYX ~ 30, 45, 60 deg
            origin_row = [1, 2, 3];
            frame = ReferenceFrame3d.from_euler(angles_rad, origin_row, 'Sequence', 'zyx', 'Units', 'rad');

            R_expected = testCase.eul2rotm_local(rad2deg(angles_rad), 'zyx', 'deg'); % Use helper
            T_expected = [R_expected, origin_row'; 0 0 0 1];

            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
        end

        function testStaticFromEulerSequenceXYZ(testCase)
            % Test from_euler using a different sequence (XYZ)
            angles_deg = [30, 45, 60]; % XYZ
            origin_row = [-1, -2, -3];
            frame = ReferenceFrame3d.from_euler(angles_deg, origin_row, 'Sequence', 'xyz', 'Units', 'deg');

            % Need a helper for xyz or implement calculation here
            % R = Rx(angles_deg(1)) * Ry(angles_deg(2)) * Rz(angles_deg(3))
            a = deg2rad(angles_deg(1)); % x
            b = deg2rad(angles_deg(2)); % y
            c = deg2rad(angles_deg(3)); % z
            Rx = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
            Ry = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
            Rz = [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1];
            R_expected = Rx * Ry * Rz; % Correct order for 'xyz' sequence

            T_expected = [R_expected, origin_row'; 0 0 0 1];

            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
        end

        function testStaticFromViewAxis(testCase)
            % Test from_view_axis basic case
            observer = [10 0 0];
            target   = [0 0 0];
            up       = [0 0 1]; % Standard Z-up
            frame = ReferenceFrame3d.from_view_axis(observer, target, up);

            % Expected Z: points from observer to target
            z_expected = (target - observer)';
            z_expected = z_expected / norm(z_expected); % Should be [-1; 0; 0]

            % Expected X: cross(target-observer, up) = cross(-X, Z) = +Y
            x_expected = cross(target - observer, up)';
            x_expected = x_expected / norm(x_expected); % Should be [0; 1; 0]

            % Expected Y: cross(Z, X) = cross(-X, Y) = -Z
            y_expected = cross(z_expected, x_expected); % Should be [0; 0; -1]

            testCase.verifyEqual(frame.origin, observer', 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.x, x_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.y, y_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.z, z_expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.R * frame.R', eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(det(frame.R), 1, 'AbsTol', testCase.Tol);
        end

        function testStaticEcefConversionsRad(testCase)
            % Test ECEF conversions using radians input
            % Using approximate values for London City Airport EGGW
            lat_deg = 51.5054; lon_deg = 0.0553; alt_m = 6;
            lat_rad = deg2rad(lat_deg); lon_rad = deg2rad(lon_deg);

            % Just checking execution and basic properties for rad input
            frame_ned_rad = ReferenceFrame3d.ecef2ned(lat_rad, lon_rad, alt_m, 'rad');
            testCase.verifyEqual(det(frame_ned_rad.R), 1, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame_ned_rad.R * frame_ned_rad.R', eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyFalse(isequal(frame_ned_rad.origin, [0;0;0])); % Origin should be non-zero

            frame_enu_rad = ReferenceFrame3d.ecef2enu(lat_rad, lon_rad, alt_m, 'rad');
            testCase.verifyEqual(det(frame_enu_rad.R), 1, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame_enu_rad.R * frame_enu_rad.R', eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyFalse(isequal(frame_enu_rad.origin, [0;0;0])); % Origin should be non-zero

            % Crude check: ENU should be NED rotated
            R_ned_to_enu = [0 1 0; 1 0 0; 0 0 -1];
            testCase.verifyEqual(frame_enu_rad.R, frame_ned_rad.R * R_ned_to_enu, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame_enu_rad.origin, frame_ned_rad.origin, 'AbsTol', testCase.Tol); % Origins should be the same
        end

        %% Property Access Tests
        function testDependentProperties(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_arb = [R_arb, origin_arb; 0 0 0 1];
            frame = ReferenceFrame3d(T_arb);

            testCase.verifyEqual(frame.R, T_arb(1:3, 1:3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.origin, T_arb(1:3, 4), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.x, T_arb(1:3, 1), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.y, T_arb(1:3, 2), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.z, T_arb(1:3, 3), 'AbsTol', testCase.Tol);
        end

        function testSetTValidation(testCase)
             frame = ReferenceFrame3d();
             T_orig = frame.T; % Get a valid T

             % Invalid size
             testCase.verifyError(@() set(frame, 'T', zeros(3,4)), ?MException);
             % Non-finite
             T_nan = T_orig; T_nan(1,1) = NaN;
             testCase.verifyError(@() set(frame, 'T', T_nan), ?MException);
             % Non-real
             T_complex = T_orig; T_complex(1,1) = 1i;
             testCase.verifyError(@() set(frame, 'T', T_complex), ?MException);

             % Non-orthonormal rotation (R*R' ~= I)
             T_bad_ortho = T_orig; T_bad_ortho(1,2) = 0.5; % Breaks orthogonality
             testCase.verifyError(@() set(frame, 'T', T_bad_ortho), ?MException);

             % Determinant ~= 1
             T_bad_det = T_orig; T_bad_det(1:3,1:3) = diag([1 1 -1]); % Determinant is -1
             testCase.verifyError(@() set(frame, 'T', T_bad_det), ?MException);

             % Bottom row not [0 0 0 1]
             T_bad_bottom = T_orig; T_bad_bottom(4,1) = 1;
             testCase.verifyError(@() frame.update(T_bad_bottom), ?MException, ...
                 'Bottom row check should fail');
        end

        %% Transformation Method Tests
        function testTranslate(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            orig_origin = frame.origin;
            dxyz = [0.1; -0.2; 0.3];

            frame.translate(dxyz);

            testCase.verifyEqual(frame.origin, orig_origin + dxyz, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.R, R_arb, 'AbsTol', testCase.Tol); % R should be unchanged
        end

        function testReposition(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            new_pos = [-5; 6; -7];

            frame.reposition(new_pos);

            testCase.verifyEqual(frame.origin, new_pos, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(frame.R, R_arb, 'AbsTol', testCase.Tol); % R should be unchanged
        end

        function testRotateDCM(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            orig_T = frame.T;
            dcm = testCase.eul2rotm_local([10, -20, 5], 'zyx', 'deg'); % Incremental rotation

            frame.rotate_dcm(dcm);

            T_rot_incr = [dcm, [0;0;0]; 0 0 0 1];
            T_expected = orig_T * T_rot_incr;

            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
        end

        function testRotateEuler(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            orig_T = frame.T;
            angles_deg = [15, -30, 45]; % ZYX incremental rotation

            frame.rotate_euler(angles_deg, 'Sequence', 'zyx', 'Units', 'deg');

            R_incr = testCase.eul2rotm_local(angles_deg, 'zyx', 'deg');
            T_rot_incr = [R_incr, [0;0;0]; 0 0 0 1];
            T_expected = orig_T * T_rot_incr;

            testCase.verifyEqual(frame.T, T_expected, 'AbsTol', testCase.Tol);
        end

        function testTranslateNonFinite(testCase)
            % Test translate with non-finite input.
            frame = ReferenceFrame3d();
            testCase.verifyError(@() frame.translate([NaN; 0; 0]), ?MException);
            testCase.verifyError(@() frame.translate([Inf; 0; 0]), ?MException);
            testCase.verifyError(@() frame.translate([0; -Inf; 0]), ?MException);
        end

        function testRepositionNonFinite(testCase)
            % Test reposition with non-finite input.
            frame = ReferenceFrame3d();
            testCase.verifyError(@() frame.reposition([NaN; 0; 0]), ?MException);
            testCase.verifyError(@() frame.reposition([Inf; 0; 0]), ?MException);
            testCase.verifyError(@() frame.reposition([0; -Inf; 0]), ?MException);
        end

        function testRotateDCMInvalidSize(testCase)
            % Test rotate_dcm with invalid input size.
            frame = ReferenceFrame3d();
            testCase.verifyError(@() frame.rotate_dcm(eye(4)), ?MException);
            testCase.verifyError(@() frame.rotate_dcm([1;2;3]), ?MException);
        end

        function testRotateEulerInvalidUnits(testCase)
            % Test rotate_euler with invalid units string.
            frame = ReferenceFrame3d();
            testCase.verifyError(@() frame.rotate_euler([10 20 30], 'Units', 'furlongs'), ...
                ?MException);
        end

         function testRotateEulerInvalidSequence(testCase)
            % Test rotate_euler with invalid sequence string.
            frame = ReferenceFrame3d();
            testCase.verifyError(@() frame.rotate_euler([10 20 30], 'Sequence', 'abc'), ...
                ?MException);
         end

        function testLocalToBaseMultipleOutputs(testCase)
            % Test local2base with multiple output arguments (x, y, z).
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            local_pts_nx3 = [1 1 1; 0 0 0; -1 2 -3];

            base_pts_expected = (R_arb * local_pts_nx3' + origin_arb)'; % Calculate expected

            % Call with multiple outputs
            [bx, by, bz] = frame.local2base(local_pts_nx3);
            base_pts_actual_xyz = [bx(:), by(:), bz(:)]; % Ensure column vectors

            testCase.verifyEqual(base_pts_actual_xyz, base_pts_expected, 'AbsTol', testCase.Tol);

            % Test with single point split output
             local_pt = [4;5;6];
             base_pt_expected_single = R_arb*local_pt + origin_arb;
             [bx_s, by_s, bz_s] = frame.local2base(local_pt'); % Input row
             testCase.verifyEqual([bx_s; by_s; bz_s], base_pt_expected_single, 'AbsTol', testCase.Tol);
        end

         function testBaseToLocalMultipleOutputs(testCase)
            % Test base2local with multiple output arguments (x, y, z).
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            base_pts_nx3 = [2 3 4; 1 1 1; 5 -1 0];

            R_inv = R_arb';
            origin_inv = -R_inv * origin_arb;
            local_pts_expected = (R_inv * base_pts_nx3' + origin_inv)'; % Calculate expected

            % Call with multiple outputs
            [lx, ly, lz] = frame.base2local(base_pts_nx3);
            local_pts_actual_xyz = [lx(:), ly(:), lz(:)]; % Ensure column vectors

            testCase.verifyEqual(local_pts_actual_xyz, local_pts_expected, 'AbsTol', testCase.Tol);

            % Test with single point split output
             base_pt = [7;8;9];
             local_pt_expected_single = R_inv*base_pt + origin_inv;
             [lx_s, ly_s, lz_s] = frame.base2local(base_pt'); % Input row
             testCase.verifyEqual([lx_s; ly_s; lz_s], local_pt_expected_single, 'AbsTol', testCase.Tol);
        end

        function testLocalToBaseInvalidArgs(testCase)
            % Test local2base with incorrect number of arguments.
            frame = ReferenceFrame3d();
            testCase.verifyError(@() frame.local2base(1, 2), ?MException); % Should trigger nargin error
        end

        function testBaseToLocalInvalidArgs(testCase)
            % Test base2local with incorrect number of arguments.
            frame = ReferenceFrame3d();
            testCase.verifyError(@() frame.base2local(1, 2), ?MException); % Should trigger nargin error
        end

        function testComposeSingleFrame(testCase)
             % Test compose with just one frame input.
             [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
             frame1 = ReferenceFrame3d(R_arb, origin_arb');

             composed_frame = compose(frame1);

             testCase.verifyEqual(composed_frame.T, frame1.T, 'AbsTol', testCase.Tol);
             testCase.verifyNotSameHandle(composed_frame, frame1); % Compose should create new object
        end

        function testComposeArrayInput(testCase)
             % Test compose when input is already an array (e.g., [f1; f2]).
            [R1, origin1] = testCase.createArbitraryRotOrigin(10, 20, 30);
            frame1 = ReferenceFrame3d(R1, origin1');
            [R2, origin2] = testCase.createArbitraryRotOrigin(40, 50, 60);
            frame2 = ReferenceFrame3d(R2, origin2');
            [R3, origin3] = testCase.createArbitraryRotOrigin(-10, -20, -30);
            frame3 = ReferenceFrame3d(R3, origin3');

            frames_array = [frame1; frame2; frame3]; % Input is column vector

            T_expected = frame1.T * frame2.T * frame3.T;

            frame_composed_method = compose(frames_array); % Pass column vector directly

            testCase.verifyEqual(frame_composed_method.T, T_expected, 'AbsTol', testCase.Tol);
        end

        function testLocalToBaseComposed(testCase)
             % Test local2base called on a composed sequence (array input)
            [R1, origin1] = testCase.createArbitraryRotOrigin(10, 20, 30);
            frame1 = ReferenceFrame3d(R1, origin1');
            [R2, origin2] = testCase.createArbitraryRotOrigin(40, 50, 60);
            frame2 = ReferenceFrame3d(R2, origin2');

            frames_array = [frame1; frame2]; % Column vector for compose logic

            local_pt = [1; 1; 1];
            T_composed = frame1.T * frame2.T;
            base_pt_expected = T_composed * [local_pt; 1];

            % Call local2base on the array itself
            base_pt_actual = frames_array.local2base(local_pt');

            testCase.verifyEqual(base_pt_actual', base_pt_expected(1:3), 'AbsTol', testCase.Tol);
        end

        function testBaseToLocalComposed(testCase)
             % Test base2local called on a composed sequence (array input)
            [R1, origin1] = testCase.createArbitraryRotOrigin(10, 20, 30);
            frame1 = ReferenceFrame3d(R1, origin1');
            [R2, origin2] = testCase.createArbitraryRotOrigin(40, 50, 60);
            frame2 = ReferenceFrame3d(R2, origin2');

            frames_array = [frame1; frame2]; % Column vector for compose logic

            base_pt = [2; 3; 4];
            T_composed = frame1.T * frame2.T;
            T_inv_composed = inv(T_composed); % Use MATLAB inv for simplicity here
            local_pt_expected = T_inv_composed * [base_pt; 1];

            % Call base2local on the array itself
            local_pt_actual = frames_array.base2local(base_pt');

            testCase.verifyEqual(local_pt_actual', local_pt_expected(1:3), 'AbsTol', testCase.Tol);
        end

        %% Coordinate Transformation Tests
        function testLocalToBase(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            local_pt_col = [1; 1; 1];
            local_pts_nx3 = [1 1 1; 0 0 0; -1 2 -3];

            % Single point test (col vector input -> col vector output via Nx3 path)
            base_pt_expected_col = R_arb * local_pt_col + origin_arb;
            base_pt_actual_nx3 = frame.local2base(local_pt_col'); % Input needs to be row
            testCase.verifyEqual(base_pt_actual_nx3', base_pt_expected_col, 'AbsTol', testCase.Tol);

            % Multiple points test (Nx3 input)
            base_pts_expected = (R_arb * local_pts_nx3' + origin_arb)'; % Equivalent math
            base_pts_actual = frame.local2base(local_pts_nx3);
            testCase.verifyEqual(base_pts_actual, base_pts_expected, 'AbsTol', testCase.Tol);

             % Multiple points test (x,y,z input)
             [bx, by, bz] = frame.local2base(local_pts_nx3(:,1), local_pts_nx3(:,2), local_pts_nx3(:,3));
             base_pts_actual_xyz = [bx, by, bz];
             testCase.verifyEqual(base_pts_actual_xyz, base_pts_expected, 'AbsTol', testCase.Tol);
        end

        function testBaseToLocal(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            base_pt_col = [2; 3; 4];
            base_pts_nx3 = [2 3 4; 1 1 1; 5 -1 0];

            % Manual inverse calculation for verification
            R_inv = R_arb';
            origin_inv = -R_inv * origin_arb;

            % Single point test (col vector input -> col vector output via Nx3 path)
            local_pt_expected_col = R_inv * base_pt_col + origin_inv;
            local_pt_actual_nx3 = frame.base2local(base_pt_col'); % Input needs to be row
            testCase.verifyEqual(local_pt_actual_nx3', local_pt_expected_col, 'AbsTol', testCase.Tol);

            % Multiple points test (Nx3 input)
            local_pts_expected = (R_inv * base_pts_nx3' + origin_inv)'; % Equivalent math
            local_pts_actual = frame.base2local(base_pts_nx3);
            testCase.verifyEqual(local_pts_actual, local_pts_expected, 'AbsTol', testCase.Tol);

             % Multiple points test (x,y,z input)
             [lx, ly, lz] = frame.base2local(base_pts_nx3(:,1), base_pts_nx3(:,2), base_pts_nx3(:,3));
             local_pts_actual_xyz = [lx, ly, lz];
             testCase.verifyEqual(local_pts_actual_xyz, local_pts_expected, 'AbsTol', testCase.Tol);
        end

        %% Composition and Inversion Tests
        function testInverse(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            frame = ReferenceFrame3d(R_arb, origin_arb');
            frame_inv = inv(frame);

            % Manual calculation of inverse
            R_inv_expected = R_arb';
            origin_inv_expected = -R_inv_expected * origin_arb;
            T_inv_expected = [R_inv_expected, origin_inv_expected; 0 0 0 1];

            testCase.verifyEqual(frame_inv.T, T_inv_expected, 'AbsTol', testCase.Tol);

            % Test F * inv(F) = I
            identity_frame = frame * frame_inv; %#ok<*MINV>
            testCase.verifyEqual(identity_frame.T, eye(4), 'AbsTol', testCase.Tol);

             % Test inv(F) * F = I
            identity_frame_2 = frame_inv * frame;
            testCase.verifyEqual(identity_frame_2.T, eye(4), 'AbsTol', testCase.Tol);
        end

        function testComposeAndMtimes(testCase)
            [R1, origin1] = testCase.createArbitraryRotOrigin();
            frame1 = ReferenceFrame3d(R1, origin1');

            [R2, origin2] = testCase.createArbitraryRotOrigin(45, 10, -60); % Different angles
            frame2 = ReferenceFrame3d(R2, origin2');

            T_expected = frame1.T * frame2.T;

            % Test mtimes (*)
            frame_composed_mtimes = frame1 * frame2;
            testCase.verifyEqual(frame_composed_mtimes.T, T_expected, 'AbsTol', testCase.Tol);

            % Test compose method
            frame_composed_method = compose(frame1, frame2);
            testCase.verifyEqual(frame_composed_method.T, T_expected, 'AbsTol', testCase.Tol);
        end

        function testComposeMultiple(testCase)
            [R1, origin1] = testCase.createArbitraryRotOrigin(10, 20, 30);
            frame1 = ReferenceFrame3d(R1, origin1');
            [R2, origin2] = testCase.createArbitraryRotOrigin(40, 50, 60);
            frame2 = ReferenceFrame3d(R2, origin2');
            [R3, origin3] = testCase.createArbitraryRotOrigin(-10, -20, -30);
            frame3 = ReferenceFrame3d(R3, origin3');

            frames_array = [frame1; frame2; frame3]; % Needs column for compose

            T_expected = frame1.T * frame2.T * frame3.T;

            frame_composed_method = compose(frames_array); % Pass column vector
            testCase.verifyEqual(frame_composed_method.T, T_expected, 'AbsTol', testCase.Tol);
        end

        %% Numeric Conversion Tests
        function testAsTransform(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_arb = [R_arb, origin_arb; 0 0 0 1];
            frame = ReferenceFrame3d(T_arb);
            testCase.verifyEqual(frame.as_transform(), T_arb, 'AbsTol', testCase.Tol);
        end

        function testAsDcm(testCase)
            [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
            T_arb = [R_arb, origin_arb; 0 0 0 1];
            frame = ReferenceFrame3d(T_arb);
            testCase.verifyEqual(frame.as_dcm(), R_arb, 'AbsTol', testCase.Tol);
        end

        function testAsEuler(testCase)
            angles_deg_in = [30, 45, 60]; % ZYX
            origin_row = [1, 2, 3];
            frame = ReferenceFrame3d.from_euler(angles_deg_in, origin_row, 'Units', 'deg');

            [yaw_deg, pitch_deg, roll_deg] = frame.as_euler('deg');
            angles_deg_out = [yaw_deg, pitch_deg, roll_deg];

            % Euler angles can have multiple representations (e.g., +/- 180).
            % Easiest verification is to convert back to rotation matrix.
            R_out = testCase.eul2rotm_local(angles_deg_out, 'zyx', 'deg');
            testCase.verifyEqual(R_out, frame.R, 'AbsTol', testCase.Tol);

            % Test radians output
             [yaw_rad, pitch_rad, roll_rad] = frame.as_euler('rad');
             testCase.verifyEqual([yaw_rad, pitch_rad, roll_rad], deg2rad(angles_deg_in), 'AbsTol', testCase.Tol);
        end

        function testAsEulerGimbalLock(testCase)
            % Test pitch = +90 deg
            angles_deg_in_p90 = [30, 90, 0]; % Yaw doesn't matter, roll is relative
            frame_p90 = ReferenceFrame3d.from_euler(angles_deg_in_p90, [0 0 0], 'Units', 'deg');
            [y_p90, p_p90, r_p90] = frame_p90.as_euler('deg');
            testCase.verifyEqual(p_p90, 90, 'AbsTol', testCase.Tol);
             % In gimbal lock at +90, yaw=0, roll = original_yaw + original_roll
            R_out_p90 = testCase.eul2rotm_local([y_p90, p_p90, r_p90], 'zyx', 'deg');
            testCase.verifyEqual(R_out_p90, frame_p90.R, 'AbsTol', testCase.Tol);


            % Test pitch = -90 deg
            angles_deg_in_n90 = [45, -90, 0]; % Yaw doesn't matter, roll is relative
            frame_n90 = ReferenceFrame3d.from_euler(angles_deg_in_n90, [0 0 0], 'Units', 'deg');
            [y_n90, p_n90, r_n90] = frame_n90.as_euler('deg');
            testCase.verifyEqual(p_n90, -90, 'AbsTol', testCase.Tol);
             % In gimbal lock at -90, yaw=0, roll = original_roll - original_yaw
            R_out_n90 = testCase.eul2rotm_local([y_n90, p_n90, r_n90], 'zyx', 'deg');
             testCase.verifyEqual(R_out_n90, frame_n90.R, 'AbsTol', testCase.Tol);
        end

        %% Copyable Test
        function testCopyMethod(testCase)
             [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
             frame_orig = ReferenceFrame3d(R_arb, origin_arb');
             frame_copy = copy(frame_orig);

             % Check properties are equal
             testCase.verifyEqual(frame_copy.T, frame_orig.T, 'AbsTol', testCase.Tol);
             % Check they are different objects
             testCase.verifyNotSameHandle(frame_orig, frame_copy);

             % Modify copy and check original is unchanged
             frame_copy.translate([1;1;1]);
             testCase.verifyNotEqual(frame_copy.T, frame_orig.T);
        end

        %% Toolbox Interoperability Tests (Conditional)
        % These tests only run if the required toolboxes are installed.

        function testSE3Conversion(testCase)
            try
                [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
                T_arb = [R_arb, origin_arb; 0 0 0 1];
                frame = ReferenceFrame3d(T_arb);
                frame_se3 = frame.se3(); % Convert to se3
    
                % Verify class
                testCase.verifyClass(frame_se3, 'se3');
                % Verify value
                testCase.verifyEqual(frame_se3.tform, T_arb, 'AbsTol', testCase.Tol);
    
                 % Test update from se3
                 frame_se3_new = se3(testCase.eul2rotm_local([5 10 15],'zyx','deg'), 'eul', origin_arb'/2);
                 frame.update(frame_se3_new);
                 testCase.verifyEqual(frame.T, frame_se3_new.tform, 'AbsTol', testCase.Tol);
            catch me
                if strcmp(me.identifier, 'MATLAB:UndefinedFunction') && contains(me.message, 'se3')
                    % user doesn't have the right toolbox; skip
                    return
                else
                    rethrow(me);
                end
            end
        end

        function testSO3Conversion(testCase)
            try
                [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
                frame = ReferenceFrame3d(R_arb, origin_arb');
                frame_so3 = frame.so3(); % Convert to so3
    
                % Verify class
                testCase.verifyClass(frame_so3, 'so3');
                % Verify value
                testCase.verifyEqual(frame_so3.rotm, R_arb, 'AbsTol', testCase.Tol);
    
                 % Test update from so3
                 frame_so3_new = so3(testCase.eul2rotm_local([-5 -10 -15],'zyx','deg'), 'rotm');
                 origin_new = [9;8;7];
                 frame.update(frame_so3_new, origin_new');
                 testCase.verifyEqual(frame.R, frame_so3_new.rotm, 'AbsTol', testCase.Tol);
                 testCase.verifyEqual(frame.origin, origin_new, 'AbsTol', testCase.Tol);
            catch me
                if strcmp(me.identifier, 'MATLAB:UndefinedFunction') && contains(me.message, 'so3')
                    % user doesn't have the right toolbox; skip
                    return
                else
                    rethrow(me);
                end
            end
        end

        function testQuaternionConversion(testCase)
            try
                [R_arb, origin_arb] = testCase.createArbitraryRotOrigin();
                frame = ReferenceFrame3d(R_arb, origin_arb');
                frame_quat = frame.quaternion(); % Convert to quaternion
    
                % Verify class
                testCase.verifyClass(frame_quat, 'quaternion');
                % Verify value by converting back to rotation matrix
                testCase.verifyEqual(rotmat(frame_quat, 'frame'), R_arb, 'AbsTol', testCase.Tol);
    
                % Test update from quaternion
                q_new = quaternion(testCase.eul2rotm_local([2 4 6],'zyx','deg'), 'rotmat', 'frame');
                origin_new = [1;4;9];
                frame.update(q_new, origin_new'); % Update uses rotmat(q,'frame') internally
                testCase.verifyEqual(frame.R, rotmat(q_new,'frame'), 'AbsTol', testCase.Tol);
                testCase.verifyEqual(frame.origin, origin_new, 'AbsTol', testCase.Tol);
            catch me
                if strcmp(me.identifier, 'MATLAB:UndefinedFunction') && contains(me.message, 'quaternion')
                    % user doesn't have the right toolbox; skip
                    return
                else
                    rethrow(me);
                end
            end
        end

    end

    methods (Access = private)
        % Helper Functions to avoid toolbox dependencies

        function [R, origin] = createArbitraryRotOrigin(testCase, ang1, ang2, ang3)
            % Creates a repeatable arbitrary rotation and origin
            if nargin < 2, ang1 = 30; end
            if nargin < 3, ang2 = -45; end
            if nargin < 4, ang3 = 60; end

            R = testCase.eul2rotm_local([ang1, ang2, ang3], 'zyx', 'deg');
            origin = [1; 2; 3]; % Column vector
        end

        function R = eul2rotm_local(~, angles, sequence, units)
            % Simple ZYX Euler to Rotation Matrix conversion.
            % Expand this if other sequences are needed for tests.
            arguments
                ~
                angles (1,3) double
                sequence (1,3) char = 'zyx'
                units (1,:) char = 'rad'
            end

            if strcmpi(units, 'deg')
                angles = angles * pi/180;
            end

            a = angles(1); % Angle for first rotation in sequence
            b = angles(2); % Angle for second rotation
            c = angles(3); % Angle for third rotation

            ca = cos(a); sa = sin(a);
            cb = cos(b); sb = sin(b);
            cc = cos(c); sc = sin(c);

            switch lower(sequence)
                case 'zyx'
                    % Rz(a) * Ry(b) * Rx(c)
                    Rx = [1 0 0; 0 cc -sc; 0 sc cc];
                    Ry = [cb 0 sb; 0 1 0; -sb 0 cb];
                    Rz = [ca -sa 0; sa ca 0; 0 0 1];
                    R = Rz * Ry * Rx;
                % Add other sequences here if needed by tests
                otherwise
                    error('Sequence ''%s'' not implemented in test helper.', sequence);
            end
        end

        function angles = rotm2eul_local(~, R, sequence, units)
             % Simple ZYX Rotation Matrix to Euler Angles conversion.
             % Matches the implementation in as_euler for verification.
             arguments
                ~
                R (3,3) double
                sequence (1,3) char = 'zyx'
                units (1,:) char = 'rad'
             end

             if ~strcmpi(sequence, 'zyx')
                 error('Sequence ''%s'' not implemented in test helper.', sequence);
             end

             % Use transpose because as_euler expects R_local_to_base'
             Rt = R';
             r11 = Rt(1,1); r12 = Rt(1,2); r13 = Rt(1,3);
             r21 = Rt(2,1); r22 = Rt(2,2); r23 = Rt(2,3);
             r33 = Rt(3,3);

             tol = 1e-12; % Tolerance close to 1

             if abs(r13) > (1 - tol)
                % Gimbal lock
                yaw = 0; % Convention: set yaw to 0
                if r13 < 0 % Pitch = -90 deg
                    pitch = -pi/2;
                    roll = atan2(r21, r22); % roll = atan2(R21, R22)
                else % Pitch = +90 deg
                    pitch = pi/2;
                    roll = -atan2(r21, r22); % roll = -atan2(R21, R22)
                end
            else
                % Normal case
                pitch = asin(-r13);
                yaw = atan2(r12, r11);
                roll = atan2(r23, r33);
            end

            angles = [yaw, pitch, roll]; % ZYX order

            if strcmpi(units, 'deg')
                angles = angles * 180/pi;
            end
        end

        function installed = isToolboxInstalled(~, toolboxName)
            % Checks if a toolbox is installed using ver.
            % Safer than exist() for toolbox classes which might be shadowed.
            v = ver;
            installed = any(strcmp(toolboxName, {v.Name}));
        end

    end % methods (Access = private)

end % classdef