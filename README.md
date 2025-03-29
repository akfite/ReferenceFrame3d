# ReferenceFrame3d

[![View on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/XXXXXX-referenceframe3d) <!-- TODO: Replace XXXXXX with File Exchange ID if submitted -->
<!-- Add other badges as needed, e.g., License, Version -->

A MATLAB class to manage complex relationships between reference frames, particularly when plotting 3D scenes.  Intended for tracking, robotics, computer vision, and related fields.

## Overview

The `ReferenceFrame3d` class encapsulates a 3D rigid body transformation, comprising both rotation and translation. A **vector** of `ReferenceFrame3d` objects represents a **transformation sequence** from the base frame of the first element to the local frame of the last element.  

This class is particularly useful for managing reference frames when plotting data.  It uses a hierarchy of efficient `hgtransform` objects to enable the user to plot data in local coordinates and have it automatically appear in the correct world position in the axes.

## Examples

### TODO

## Features

*   **Representation:** Stores pose as a 4x4 homogeneous transformation matrix (`T`).
*   **Convenience Accessors:** Easily get the 3x3 rotation matrix (`R`), 3x1 translation vector (`t`), and orthogonal basis vectors (`x`, `y`, `z`).
*   **Flexible Construction:**
    *   Default identity frame.
    *   From a 4x4 `T` matrix.
    *   From a 3x3 rotation matrix (`R` or DCM) and an origin vector (`t`).
    *   From `quaternion`, `se3`, or `so3` objects (requires Navigation Toolbox™ / Robotics System Toolbox™).
    *   Statically from a point and a normal vector to define a plane (`from_point_normal`).
*   **Coordinate Transformations:** `local2base` and `base2local` methods for transforming vectors or sets of points (Nx3 arrays or separate x, y, z inputs).
*   **Frame Manipulation:**
    *   `translate`: Apply incremental translation.
    *   `reposition`: Set absolute position (origin).
    *   `rotate`: Apply incremental rotation using a 3x3 DCM.
    *   `rotate_euler` / `rotate_eulerd`: Apply incremental ZYX Euler angle rotation (radians/degrees).
*   **Composition & Inverse:**
    *   `compose`: Chain multiple `ReferenceFrame3d` objects (`T_total = T1 * T2 * ...`).
    *   `mtimes` (`*`): Overloaded multiplication for intuitive composition (`frame_total = frame1 * frame2`).
    *   `inv`: Compute the inverse transformation.
*   **Numeric Conversions:**
    *   `as_transform()`: Get the 4x4 `T` matrix.
    *   `as_dcm()`: Get the 3x3 `R` matrix.
    *   `as_euler()` / `as_eulerd()`: Get ZYX Euler angles (radians/degrees).
*   **Toolbox Interoperability:**
    *   `se3()`, `so3()`, `quaternion()`: Convert to corresponding objects (requires Navigation Toolbox™ / Robotics System Toolbox™).
*   **Visualization:**
    *   `plot()`: Plot the frame's origin and basis vectors in a given axes. Highly customizable (colors, line styles, lengths, arrowheads).
    *   `show()`: Convenience method to plot in a new figure.
    *   `draw_plane()`: Draw a configurable planar surface aligned with two of the frame's basis vectors.
    *   `hgtransform()`: Creates/retrieves the underlying `hgtransform` object for efficient hierarchical plotting and updates. Graphics automatically update when `T` is modified.
    *   `clear()`: Deletes associated graphics objects.
*   **Object Handling:**
    *   Supports deep copying using the `copy()` method, which also copies graphics handles.
*   **Validation:** Includes internal checks (`validate_transform`) to ensure the rotation part of `T` remains valid (orthogonal, det(R) = 1).

## Requirements

*   MATLAB R2021a or later (uses `arguments` block syntax). Earlier versions might work with modifications.
*   **Optional:** Navigation Toolbox™ or Robotics System Toolbox™ for `se3`, `so3`, and `quaternion` conversion methods.

## Installation

Add the folder containing `ReferenceFrame3d.m` to your MATLAB path.

## Usage Examples

### 1. Creating Frames

```matlab
% Default identity frame (origin at [0,0,0], no rotation)
frame0 = ReferenceFrame3d();

% From a 4x4 homogeneous transform matrix
T = [0 1 0 5;  % Rotation: 90 deg around Z, Translation: [5; -2; 1]
    -1 0 0 -2;
     0 0 1 1;
     0 0 0 1];
frame1 = ReferenceFrame3d(T);

% From a 3x3 Rotation Matrix (DCM) and an origin vector
R = eul2rotm([pi/4, 0, 0]); % 45 deg rotation around X
origin = [1; 2; 3];
frame2 = ReferenceFrame3d(R, origin);

% Setup an existing frame object
frame3 = ReferenceFrame3d();
frame3.setup(R, origin); % Same as frame2

% From a point and normal (creates a frame representing a plane)
pt = [1 1 1];
n = [0 0 1]; % Normal pointing up Z-axis
plane_frame = ReferenceFrame3d.from_point_normal(pt, n);

% --- Requires Navigation/Robotics System Toolbox ---
% From a quaternion and origin
q = quaternion([30, 45, 60], 'eulerd', 'ZYX', 'frame');
origin_q = [ -1; 0; 5];
frame_q = ReferenceFrame3d(q, origin_q);

% From an se3 object
tform_se3 = se3(T);
frame_se3 = ReferenceFrame3d(tform_se3);
```

### 2. Accessing Properties

```matlab
T_mat = frame1.T;        % Get the 4x4 matrix
R_mat = frame1.R;        % Get the 3x3 rotation matrix
t_vec = frame1.t;        % Get the 3x1 translation vector (origin)
x_basis = frame1.x;      % Get the x-axis basis vector in the base frame
y_basis = frame1.y;      % Get the y-axis basis vector in the base frame
z_basis = frame1.z;      % Get the z-axis basis vector in the base frame

disp('Frame 1 Origin:');
disp(frame1.t);

disp('Frame 1 Z-axis:');
disp(frame1.z);
```

### 3. Transforming Points

```matlab
% Define points in the local frame of frame1
p_local = [1; 0; 0]; % Point along frame1's local x-axis
points_local = [1 0 0; 0 1 0; 0 0 1; 1 1 1]; % Nx3 array

% Transform to base frame coordinates
p_base = frame1.local2base(p_local);
points_base = frame1.local2base(points_local);

fprintf('Local point [1;0;0] in base frame: [%.2f, %.2f, %.2f]\n', p_base);

% Transform points from base frame to frame1's local coordinates
p_local_check = frame1.base2local(p_base);
points_local_check = frame1.base2local(points_base);

fprintf('Base point back to local frame: [%.2f, %.2f, %.2f]\n', p_local_check);

% Using separate x,y,z inputs/outputs
x_local = [1; 0; 0; 1];
y_local = [0; 1; 0; 1];
z_local = [0; 0; 1; 1];
[x_base, y_base, z_base] = frame1.local2base(x_local, y_local, z_local);
```

### 4. Modifying the Frame

```matlab
frame_mod = ReferenceFrame3d(eye(3), [1; 1; 1]);
disp('Initial Position:'); disp(frame_mod.t');

% Apply incremental translation
frame_mod.translate([0.5; -0.5; 0]);
disp('After translate([0.5; -0.5; 0]):'); disp(frame_mod.t');

% Set absolute position
frame_mod.reposition([5; 5; 5]);
disp('After reposition([5; 5; 5]):'); disp(frame_mod.t');

% Apply incremental rotation (30 degrees around current Z-axis)
R_inc = eul2rotm([0, 0, pi/6]);
frame_mod.rotate(R_inc);
disp('After 30 deg Z rotation (DCM):'); disp(frame_mod.as_eulerd());

% Apply incremental rotation using Euler angles (degrees)
frame_mod.rotate_eulerd(0, 0, 30); % Another 30 deg around current Z
disp('After 30 deg Z rotation (Euler):'); disp(frame_mod.as_eulerd());
```

### 5. Composition and Inverse

```matlab
% Define two frames
frameA = ReferenceFrame3d(eul2rotm([0 0 pi/4]), [1; 0; 0]); % 45deg Z rot, translate X=1
frameB = ReferenceFrame3d(eul2rotm([pi/4 0 0]), [0; 1; 0]); % 45deg X rot, translate Y=1

% Compose: Transform from frameB's local coords to base frame
% T_A_B = T_W_A * T_A_B (where W=World/Base)
frame_comp = compose(frameA, frameB); % Equivalent to frame_comp = frameA * frameB;
disp('Composed Frame (A * B):');
disp(frame_comp); % Shows origin and Euler angles

% Define a point in frameB's local coordinates
p_B = [1; 0; 0];

% Transform p_B directly to base frame using composed frame
p_base_comp = frame_comp.local2base(p_B);

% Transform step-by-step for verification
p_A = frameB.local2base(p_B); % p_B relative to frameA
p_base_step = frameA.local2base(p_A); % p_A relative to base

fprintf('Composed transform result: [%.2f, %.2f, %.2f]\n', p_base_comp);
fprintf('Step-by-step transform result: [%.2f, %.2f, %.2f]\n', p_base_step);

% Calculate the inverse of frameA (transform from Base to A)
frameA_inv = inv(frameA);
disp('Inverse of Frame A:');
disp(frameA_inv);

% Verify: A * inv(A) should be identity
frame_identity = frameA * frameA_inv;
disp('A * inv(A) (should be close to identity):');
disp(frame_identity.T);
```

### 6. Visualization

```matlab
% Create a figure and axes
figure;
ax = axes('DataAspectRatio', [1 1 1]);
grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);

% Plot the base frame (identity)
base_frame = ReferenceFrame3d();
base_frame.plot('Parent', ax, 'LineWidth', 2, 'LineLength', 0.5);

% Create and plot another frame relative to the base
frame1 = ReferenceFrame3d(eul2rotm([0 0 pi/4]), [1; 1; 0]);
frame1.plot('Parent', ax, 'LineWidth', 2, 'LineLength', 1);

% Create a frame relative to frame1 (hierarchical plotting)
frame2 = ReferenceFrame3d(eul2rotm([pi/4 0 0]), [0.5; 0; 0]); % Relative offset along frame1's X
% Parent plotting to frame1's hgtransform handle
frame2.plot('Parent', frame1); % Auto-uses frame1.hgtransform if plotted

% Modify frame1 and see graphics update automatically
pause(1);
frame1.translate([0; 0; 0.5]); % Move frame1 (and frame2 with it) up
title('Frames after translation');
pause(1);
frame1.rotate_eulerd(0, 0, 45); % Rotate frame1 (and frame2 with it)
title('Frames after rotation');

% Draw a plane associated with frame2's XY plane
frame2.draw_plane('Slice', 'xy', 'Size', [0.4 0.6], 'FaceColor', 'c', 'FaceAlpha', 0.5);

% Use show() for a quick plot in a new figure
frame_show = ReferenceFrame3d(eye(3), [2; 0; 0]);
frame_show.show(); % Creates new figure and plots
title('Plot using show()');
```

### 7. Conversion

```matlab
frame = ReferenceFrame3d(eul2rotm([pi/6, pi/4, pi/3]), [1; 2; 3]);

T4x4 = frame.as_transform();
R3x3 = frame.as_dcm();
[roll_rad, pitch_rad, yaw_rad] = frame.as_euler();
[roll_deg, pitch_deg, yaw_deg] = frame.as_eulerd();

fprintf('Euler Angles (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n', roll_deg, pitch_deg, yaw_deg);

% --- Requires Navigation/Robotics System Toolbox ---
try
    frame_se3 = frame.se3();
    frame_so3 = frame.so3();
    frame_quat = frame.quaternion();
    disp('Converted to se3:'); disp(frame_se3);
    disp('Converted to quaternion:'); disp(frame_quat);
catch ME
    warning('Navigation/Robotics System Toolbox specific methods failed: %s', ME.message);
end
```

### 8. Copying Objects

```matlab
frame_orig = ReferenceFrame3d(eul2rotm([0 0 pi/6]), [1; 0; 0]);
frame_copy = copy(frame_orig);

% Modify the copy - the original remains unchanged
frame_copy.translate([0; 1; 0]);

disp('Original frame origin:'); disp(frame_orig.t');
disp('Copied frame origin:'); disp(frame_copy.t');
```

## Properties

### Read/Write
*   `T` (4x4 double): The homogeneous transformation matrix.
    * public access
    * transform is validated before setting
    * updates hgtransform object when set (if one exists)
    * recommend setting this property via method `setup(rot, origin)` or other methods for rotation, translation, etc

### Read-Only
*   `R` (3x3 double, Dependent): Rotation submatrix.
*   `t` (3x1 double, Dependent): Translation vector (origin).
*   `x`, `y`, `z` (3x1 double, Dependent): Basis vectors.

## Methods
### Construction

*   `ReferenceFrame3d()`: Default constructor (`eye(4)`).
*   `ReferenceFrame3d(rot)`: Rotation-only constructor.
*   `ReferenceFrame3d(rot, origin)`: Rotation & translation.
*   `setup(rot, origin)`: Configure an existing object.

> `rot` argument can be 4x4 T, 3x3 R, `quaternion`, `se3`, or `so3`.

#### Static Constructors (Utility)
*   `ReferenceFrame3d.from_point_normal(point, normal)`: Create a frame to represent a plane using `normal` as `+z` and `point` as the origin. 
*   `ReferenceFrame3d.from_coplanar_vectors(v1, v2)`: Create a frame to represent a plane using two coplanar vectors.

### Transformations

*   `local2base(vec)` or `local2base(x,y,z)`: Transform local coordinates to base.
*   `base2local(vec)` or `base2local(x,y,z)`: Transform base coordinates to local.
*   `translate(dxyz)`: Apply incremental translation.
*   `reposition(new_pos)`: Set absolute origin.
*   `rotate(dcm)`: Apply incremental rotation (3x3 DCM).
*   `rotate_euler(r, p, y)`: Apply incremental ZYX Euler rotation (radians).
*   `rotate_eulerd(r, p, y)`: Apply incremental ZYX Euler rotation (degrees).
*   `compose(frame1, frame2, ...)`: Compose a sequence of frames.
*   `mtimes(other)` or `*`: Overload for composition.
*   `inv()`: Compute inverse transformation.

### Numeric Representations

*   `as_transform()`: Return 4x4 `T`.
*   `as_dcm()`: Return 3x3 `R`.
*   `as_euler()`: Return `[roll, pitch, yaw]` in radians.
*   `as_eulerd()`: Return `[roll, pitch, yaw]` in degrees.

### Toolbox Conversions (Optional)

*   `se3()`: Convert to `se3` object.
*   `so3()`: Convert to `so3` object.
*   `quaternion()`: Convert rotation to `quaternion` object.

### Graphics

*   `show()`: Plot the reference frame(s) into a new figure.
*   `plot()`: Plot the basis vectors for one or more frames to an existing axis.  An array of frames is plotted in an hgtransform hierarchy where the first frame is the root.
*   `hgtransform(parent)`: Get or create the underlying `hgtransform` graphics object.  An array of frames creates an hgtransform hierarchy where the first frame is the root.  When plotting, parent your handles to the `obj.hgtransform()` that defines the coordinate frame your data is expressed in.
*   `draw_plane()`: Draw a configurable plane defined by a pair of basis vectors for the frame (e.g. 'xy')
*   `clear()`: Delete all graphics associated with the object.

### Other

*   `copy()`: Create a deep copy of the object, including handle graphics (if applicable)
*   `intersect_plane()`: Calculate ray-plane intersection.

## License

Distributed under the MIT License. See `LICENSE` file for more information.
