^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package off_highway_uss
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2024-06-04)
------------------

0.6.0 (2024-05-14)
------------------
* Switch from deprecated signature to const shared_ptr reference while still supporting efficient intra-process communication
  See https://github.com/ros2/rclcpp/pull/1598.
* Convert USS driver from node to component
* Add launch argument for parameters and unify launch file names
* Contributors: Robin Petereit, Sarah Huber

0.5.1 (2024-03-27)
------------------
* Include what you use
* Remove unused variable
* Properly include rclcpp/executors.hpp when using spin_some
* Disable irrelevant warnings for Clang
  Anonymous structs are used in PCL for type punning:
  https://github.com/PointCloudLibrary/pcl/issues/2303
  Subobject braces are not needed:
  https://bugs.llvm.org/show_bug.cgi?id=21629
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25137
* Contributors: Ramon Wijnands, Robin Petereit
