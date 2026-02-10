.. _mola_lo_state_estimators:

============================================
State estimators in MOLA-LO: usage and authoring
============================================

This guide documents:

* How to use the built-in state estimators with MOLA-LO.
* How to select estimator outputs for ROS 2 publication.
* What to implement when creating a new estimator module.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

Built-in estimator options
--------------------------

The default launch files select one of two estimator classes:

* ``mola::state_estimation_simple::StateEstimationSimple``
* ``mola::state_estimation_smoother::StateEstimationSmoother``

Selection is controlled by launch argument ``use_state_estimator`` in
``ros2-launchs/ros2-lidar-odometry.launch.py``, which sets the
``MOLA_STATE_ESTIMATOR`` environment variable.

Parameter files
---------------

By default, estimator settings are loaded from:

* ``state-estimator-params/state-estimation-simple.yaml``
* ``state-estimator-params/state-estimation-smoother.yaml``

Common tunable parameters include process-noise values, planar-motion
constraints, and initial twist estimates.

How to run with smoother estimator
----------------------------------

Example:

.. code-block:: bash

   ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      use_state_estimator:=true \
      state_estimator_config_yaml:=../state-estimator-params/state-estimation-smoother.yaml

How estimator outputs are exposed to ROS 2
------------------------------------------

The launch file sets:

* ``MOLA_LOCALIZATION_PUBLISH_TF_SOURCE``
* ``MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE``

When ``use_state_estimator=true``, both default to ``state_estimator``;
otherwise they default to ``lidar_odometry``.

This means you can switch between raw LO output and fused estimator output
without changing code.

How estimators cooperate with LO internally
-------------------------------------------

In tests and real deployments, LO and estimator modules are instantiated in the
same module container so LO can find/use the estimator.

Reference implementation pattern appears in:
``test/test_lidar_odometry_rosbag2.cpp`` and
``test/test_lidar_odometry_rawlog.cpp``.

Writing a new state estimator module
------------------------------------

Recommended design checklist:

1. Implement a MOLA executable module class (initialize + update loop).
2. Subscribe to motion/sensor observations you plan to fuse (IMU, odometry,
   LO poses, GNSS, etc.).
3. Publish localization estimates as MOLA localization observations.
4. Provide a YAML-driven parameter block (noise, priors, constraints,
   publication rate).
5. Register and export the module so it can be instantiated by type string in
   launch YAML.

Integration into ``lidar_odometry_ros2.yaml``:

.. code-block:: yaml

   - name: state_estimation
     type: my_namespace::MyStateEstimator
     raw_data_source: "ros2_bridge"
     execution_rate: 20
     params: "../state-estimator-params/my-state-estimator.yaml"

Practical migration strategy
----------------------------

When bringing a new estimator online:

* First run with LO publishing to ROS 2 and estimator running in parallel.
* Compare trajectories and delays.
* Switch bridge publication source to your estimator once stable.
* Keep LO output available as fallback source during validation.

See also
--------

* :ref:`mola_lo_architecture`
* :ref:`mola_lo_localization_plugins`
