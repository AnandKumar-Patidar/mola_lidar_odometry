.. _mola_lo_architecture:

=============================================
MOLA-LO architecture and data-flow reference
=============================================

This page complements :ref:`mola_lidar_odometry` with a practical architecture
view focused on what gets connected in a real system.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

High-level module architecture
------------------------------

A typical ROS 2 deployment in this repository uses three main MOLA modules:

* ``mola::LidarOdometry`` (module name: ``lidar_odom``), which estimates motion,
  builds/uses local maps, and can do initial localization/relocalization.
* ``mola::state_estimation_*`` (module name: ``state_estimation``), which fuses
  available motion cues and provides a filtered state trajectory.
* ``mola::BridgeROS2`` (module name: ``ros2_bridge``), which imports sensor data
  from ROS 2 and publishes localization outputs back to ROS 2.

See the default ROS 2 launch system YAML for this exact arrangement.

Reference: ``mola-cli-launchs/lidar_odometry_ros2.yaml``.

Execution model
---------------

The default ROS 2 launch uses a single MOLA process (``mola-cli``) that hosts
all modules, with each module receiving observations from the bridge and
publishing outputs independently.

Important consequences:

* Relocalization requests from ROS 2 are broadcast by the bridge to any module
  implementing ``mola::Relocalization``.
* ``LidarOdometry`` and state estimators can run together and exchange
  information through the shared container.
* Output selection (which module publishes TF/odometry to ROS 2) is configured
  in bridge parameters, not hardcoded.

Data flow breakdown
-------------------

1. ROS 2 topics are subscribed in ``BridgeROS2.params.subscribe``.
2. Incoming messages are transformed into MOLA observations.
3. ``LidarOdometry`` consumes LiDAR (and optional IMU/GNSS) observations.
4. A state estimator consumes motion observations and publishes filtered state.
5. ``BridgeROS2`` republishes selected localization source as TF and/or odometry.

Key configuration pivots
------------------------

``LidarOdometry``
^^^^^^^^^^^^^^^^^

* ``raw_data_source: "ros2_bridge"`` links the module to ROS input.
* ``params: ".../pipelines/lidar3d-default.yaml"`` selects the LO pipeline.
* Pipeline internals define filtering, registration, local-map updates,
  and initial localization mode.

``State estimation``
^^^^^^^^^^^^^^^^^^^^

* ``type`` can be either simple or smoother estimator class.
* ``execution_rate`` controls publication rate for estimator outputs
  (mainly relevant for smoother).
* ``params`` points to one of the state estimator YAML parameter files.

``BridgeROS2``
^^^^^^^^^^^^^^

* ``publish_tf_from_slam_source`` and ``publish_odometry_msgs_from_slam_source``
  decide which module output is exposed to ROS 2.
* ``publish_localization_following_rep105`` toggles direct map->base_link vs
  REP-105 map->odom->base_link publication.
* ``forward_ros_tf_as_mola_odometry_observations`` allows importing external
  odometry (for example from another subsystem) into MOLA.

Where to customize architecture
-------------------------------

* For algorithmic behavior (ICP, map update, deskew, initial localization):
  customize files under ``pipelines/``.
* For ROS I/O, frame IDs, and publication behavior:
  customize ``mola-cli-launchs/lidar_odometry_ros2.yaml`` or set ROS 2 launch
  arguments/environment variables.
* For estimator behavior (noise models, planar constraints, windowing):
  customize files under ``state-estimator-params/``.

See also
--------

* :ref:`mola_lo_state_estimators`
* :ref:`mola_lo_localization_plugins`
* :ref:`mola_lo_fastlio_molareloc`
