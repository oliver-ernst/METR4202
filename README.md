### Example
```
[output1, output2, ...] = function_name(input1, input2, ...)
% these are the details for the function, feel free to break these big functions into smaller functions.
% smaller functions are easier to test and debug
```
## Functions:
```
[intrinsics, extrinsics] = calibrate(camera_img_set) \n
% Takes in the given camera image set and returns the intrinsics and extrinsics of the camera
% Input:   camera_img_set - a set of 1280x1024 RGB images in a matrix of cells with size (n,1)
%                         - each cell is it's own RGB image and has an inner array with size (1280,1024,3)
% Outputs: intrinsics     - SPEC REQUIRED
%          extrinsics     - SPEC REQUIRED
```
```
[top_down_img] = transform_scene(intrinsics, extrinsics, scene_img)
% Takes the given camera intrinsics and given scene image and transforms the image to a top down view of the scene using given 
% extrinsics, crops the image to the defined workspace
% Inputs:  intrinsics     - SPEC REQUIRED
%          extrinsics     - SPEC REQUIRED
%          scene_img      - unedited RGB scene image straight from ximera camera, read into array with size (1280,1024,3)
% Outputs: top_down_img   - transformed scene image as a top down view of the workspace (WORK OUT SIZE)
```
```
[cyl,cap,clutter_bnds] = detect_scene(top_down_img)
% Takes the transformed top down scene image and returns the boundary locations for the cylinder, pen cap and clutter.
% Inputs:  top_down_img   - transformed scene image as a top down view of the workspace (WORK OUT SIZE)
% Outputs: cyl            - 2 cell object, 1st cell has the centre point of the cylinder as (x,y), 2nd cell has all boundary points for
%                           object as size (n,2) array of (x,y) points
%          cap            - same spec as cyl
%          clutter_bnds   - locations of the boundary points of the n amount of clutter objects in scene.  Array of (n,1) cells of
%                           boundary points.  These have a increased radius for safety added.
```
```
[path_points] = plan_motion(cyl,cap,clutter_bnds)
% Does motion planning given the locations of the cylinder, pen cap, and clutter in the scene
% Inputs:  cyl            - 2 cell object, 1st cell has the centre point of the cylinder as (x,y), 2nd cell has all boundary points for
%                           object as size (n,2) array of (x,y) points
%          cap            - same spec as cyl
%          clutter_bnds   - locations of the boundary points of the n amount of clutter objects in scene.  Array of (n,1) cells of
%                           boundary points.  These have a increased radius for safety added.
% Outputs: path_points    - Array of n points of which the arm needs to move between to get from the cylinder to the pen cap.  First point 
%                           is the Cylinder location, final point is the cap location.
```
