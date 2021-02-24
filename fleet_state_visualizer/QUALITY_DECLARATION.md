This document is a declaration of software quality for the `fleet_state_visualizer` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `fleet_state_visualizer` Quality Declaration

The package `fleet_state_visualizer` claims to be in the **Quality Level 4** category.

Below are the detailed rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`fleet_state_visualizer` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`fleet_state_visualizer` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

`fleet_state_visualizer` does not have a public API.

### API Stability Policy [1.iv]

`fleet_state_visualizer` does not have a public API.

### ABI Stability Policy [1.v]

`fleet_state_visualizer` does not have a public API.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

`fleet_state_visualizer` does not have a public API.

## Change Control Process [2]

`fleet_state_visualizer` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`fleet_state_visualizer` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`fleet_state_visualizer` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.
The CI checks only that the package builds.
The most recent CI results can be seen on [the workflow page](https://github.com/osrf/rmf_schedule_visualizer/actions).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`fleet_state_visualizer` is documented in the [parent repository's README.md file](https://github.com/osrf/rmf_schedule_visualizer/blob/master/README.md).

### Public API Documentation [3.ii]

`fleet_state_visualizer` does not have a public API.

### License [3.iii]

The license for `fleet_state_visualizer` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

### Copyright Statement [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `fleet_state_visualizer`.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 4 lists.

## Testing [4]

### Feature Testing [4.i]

`fleet_state_visualizer` does not have any tests.

### Public API Testing [4.ii]

`fleet_state_visualizer` does not have a public API.

### Coverage [4.iii]

`fleet_state_visualizer` does not track coverage statistics.

### Performance [4.iv]

`fleet_state_visualizer` does not have performance tests.

### Linters and Static Analysis [4.v]

`fleet_state_visualizer` does not use any linters.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

`fleet_state_visualizer` has the following direct runtime ROS dependencies.

#### rmf_fleet_msgs

`rmf_fleet_msgs` is [**Quality Level 3**](https://github.com/osrf/rmf_core/blob/master/rmf_fleet_msgs/QUALITY_DECLARATION.md).

#### visualization_msgs

`visualization_msgs` is [**Quality Level 3**](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/QUALITY_DECLARATION.md).

#### rclpy

`rclpy` does not declare a quality level.
It is assumed to be **Quality Level 3** based on its wide-spread use, use of change control, use of CI, and use of testing.

#### std_msgs

`std_msgs` is [**Quality Level 3**](https://github.com/ros2/common_interfaces/blob/master/std_msgs/QUALITY_DECLARATION.md).

#### rmf_traffic_msgs

`rmf_traffic_msgs` is [**Quality Level 3**](https://github.com/osrf/rmf_core/blob/master/rmf_traffic_msgs/QUALITY_DECLARATION.md).

#### ament_index_python

`ament_index_python` is [**Quality Level 4**](https://github.com/ament/ament_index/blob/master/ament_index_python/QUALITY_DECLARATION.md).

#### building_map_msgs

`building_map_msgs` is [**Quality Level 3**](https://github.com/osrf/traffic_editor/blob/master/building_map_msgs/QUALITY_DECLARATION.md).

#### rmf_schedule_visualizer_msgs

`rmf_schedule_visualizer_msgs` is [**Quality Level 3**](https://github.com/osrf/rmf_state_visualizer/blob/master/rmf_schedule_visualizer_msgs/QUALITY_DECLARATION.md).

### Optional Direct Runtime ROS Dependencies [5.ii]

`fleet_state_visualizer` does not have any optional direct runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`fleet_state_visualizer` does not have any direct runtime non-ROS dependencies.

## Platform Support [6]

`fleet_state_visualizer` does not support all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
`fleet_state_visualizer` supports ROS Eloquent and ROS Foxy.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
