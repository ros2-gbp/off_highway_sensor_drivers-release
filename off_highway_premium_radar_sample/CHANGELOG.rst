^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_premium_radar_sample
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2024-06-04)
------------------

0.6.0 (2024-05-14)
------------------
* Rename premium radar driver to sample
  Currently, the delivered Radar Off-Highway Premium and its firmware are just samples.
  Separate package will be created for series firmware.
* Switch from deprecated signature to const shared_ptr reference while still supporting efficient intra-process communication
  See https://github.com/ros2/rclcpp/pull/1598.
* Interpret azimuth and elevation angle of premium radar locations as cone coordinates
* Update executable names for premium radar in launch
* Convert premium radar from node to component
* Remove redundant normal in pcl
* Contributors: Robin Petereit, Sarah Huber

0.5.1 (2024-03-27)
------------------
* Disable irrelevant warnings for Clang
  Anonymous structs are used in PCL for type punning:
  https://github.com/PointCloudLibrary/pcl/issues/2303
  Subobject braces are not needed:
  https://bugs.llvm.org/show_bug.cgi?id=21629
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25137
* Remove unnecessary self assignment
* Clarify operation order
* Remove C style cast
* Use explicit initialization
* Remove unused variables
* Add virtual destructor for base class
* Contributors: Robin Petereit
