# ReferenceFrame3d

[![View on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/XXXXXX-referenceframe3d) <!-- TODO: Replace XXXXXX with File Exchange ID if submitted -->
<!-- Add other badges as needed, e.g., License, Version -->

A MATLAB class to manage complex relationships between reference frames, particularly when plotting 3D scenes.  Intended for tracking, robotics, computer vision, and related fields.

<img src="./assets/01_demo.gif" autoplay="true"/>

## Overview

The `ReferenceFrame3d` class encapsulates a 3D rigid body transformation, comprising both rotation and translation. A **vector** of `ReferenceFrame3d` objects represents a **transformation sequence** from the base frame of the first element to the local frame of the last element.  

This class is particularly useful for managing reference frames when plotting data.  It uses a hierarchy of efficient `hgtransform` objects to enable the user to plot data in local coordinates and have it automatically appear in the correct world position in the axes.

## Getting Started

### Create and plot a reference frame

```matlab
frame = ReferenceFrame3d(eye(3), [0 0 0])
frame.show()
```

### Create a relationship between two reference frames

1. Create a `robot` reference frame relative to the `world`.
```matlab
world = ReferenceFrame3d(eye(3), [0 0 0]);
robot = ReferenceFrame3d();
robot.reposition([1 2 0]); % offset in x and y
robot.rotate_eulerd(0, 0, 45); % turn 45 degrees (yaw)
show([world robot]);
```
> `show([world robot])` is creating the hgtransform hierarchy (the relationship between the frames).  In practice, you'll probably prefer to call `hgtransform([world robot ...])` to create the transforms in your own axes.

Now, let's plot some data into the local coordinates of each of these frames.  We'll plot the same vector `[1 1 0]` parented to the tranform for each object and see where it lands. 

```matlab
plot3(world.hgtransform(), [0 1], [0 1], [0 0 ], 'k-', 'LineWidth', 2)
plot3(robot.hgtransform(), [0 1], [0 1], [0 0 ], 'k--', 'LineWidth', 2)
```

## Practical Examples

### Plot ECEF data to an axes 

### Creating a LiDAR point cloud viewer

### 

## Properties

### Read/Write
*   `T` (4x4 double): The homogeneous transformation matrix.
    * public access
    * transform is validated before setting
    * updates hgtransform object when set (if one exists)
    * recommend setting this property via method `setup(rot, origin)` or other methods for rotation, translation, etc

### Read-Only
*   `R` (3x3 double, Dependent): Rotation submatrix.
*   `origin` (3x1 double, Dependent): Translation vector.
*   `x`, `y`, `z` (3x1 double, Dependent): Basis vectors.

## Methods
### Construction

*   `ReferenceFrame3d()`: Default constructor (`eye(4)`).
*   `ReferenceFrame3d(rot)`: Rotation-only constructor.
*   `ReferenceFrame3d(rot, origin)`: Rotation & translation.
*   `setup(rot, origin)`: Configure an existing object.

> `rot` argument can be 4x4 T, 3x3 R, `ReferenceFrame3d`, `quaternion`, `se3`, or `so3`.

#### Static Constructors (Utility)
*   `ReferenceFrame3d.from_point_normal(point, normal)`: Create a frame to represent a plane using `normal` as `+z` and `point` as the origin. 
*   `ReferenceFrame3d.from_coplanar_vectors(v1, v2, origin)`: Create a frame to represent a plane using two coplanar vectors.

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
    - `ReferenceFrame3d(other_frame)` to shallow copy
*   `intersect_plane()`: Calculate ray-plane intersection.

## License

Distributed under the MIT License. See `LICENSE` file for more information.
