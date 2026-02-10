.. _mola_lo_fastlio_molareloc:

=====================================================================
Using FAST-LIO odometry as source + estimator fusion + MOLA relocalize
=====================================================================

This recipe explains how to combine:

* External odometry from FAST-LIO (published in ROS TF as ``odom->base_link``),
* MOLA-LO localization against a prebuilt map,
* MOLA state estimator fusion for smoother final localization output.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

Target architecture
-------------------

1. FAST-LIO provides high-rate local odometry in ROS 2 TF.
2. MOLA Bridge imports that TF odometry into MOLA observations.
3. MOLA-LO runs map-based localization (with preloaded map/simplemap).
4. State estimator fuses available cues and provides final pose stream.
5. Bridge publishes fused localization to ROS TF and/or odometry.

Prerequisites
-------------

* FAST-LIO (or equivalent) publishes a valid ``odom->base_link`` TF.
* A prebuilt map file is available:

  * metric map ``.mm`` for local-map preload,
  * optional keyframe ``.simplemap`` for localization/multisession workflows.

Recommended ROS 2 launch command
--------------------------------

.. code-block:: bash

   ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      use_state_estimator:=true \
      forward_ros_tf_odom_to_mola:=true \
      mola_tf_estimated_odometry:=odom \
      mola_tf_base_link:=base_link \
      mola_initial_map_mm_file:=/ABS/PATH/prebuilt_map.mm \
      mola_initial_map_sm_file:=/ABS/PATH/prebuilt_keyframes.simplemap

What each argument does
-----------------------

* ``forward_ros_tf_odom_to_mola:=true`` enables importing
  ``odom->base_link`` as MOLA odometry observations.
* ``mola_tf_estimated_odometry`` sets which TF frame is read as odometry source.
* ``mola_initial_map_mm_file`` maps to env var ``MOLA_LOAD_MM`` used by LO
  pipelines to preload metric maps.
* ``mola_initial_map_sm_file`` maps to env var ``MOLA_LOAD_SM`` used by LO
  pipelines to preload simple-maps.
* ``use_state_estimator:=true`` selects the smoother estimator class and routes
  default ROS publication source to ``state_estimator``.

Initial localization behavior
-----------------------------

The startup relocalization mode is controlled by
``initial_localization.method`` in the pipeline (via
``MOLA_LO_INITIAL_LOCALIZATION_METHOD``).

Common choices include fixed-pose initialization and alternative modes described
in the MOLA ROS 2 API docs.

For map-localization workflows, ensure the chosen initialization method is
consistent with available priors (fixed prior, IMU, GNSS, or external pose
initialization through ``/initialpose``).

Output publication strategy
---------------------------

To publish fused estimator output, keep (or set):

* ``MOLA_LOCALIZATION_PUBLISH_TF_SOURCE=state_estimator``
* ``MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE=state_estimator``

If you need debugging comparisons, temporarily switch either source to
``lidar_odom`` and compare trajectories.

Troubleshooting
---------------

* **No imported odometry:** verify FAST-LIO is publishing ``odom->base_link``
  and frame names match launch arguments.
* **No map localization effect:** verify map files exist and were loaded through
  ``mola_initial_map_mm_file`` / ``mola_initial_map_sm_file``.
* **TF inconsistencies:** check whether REP-105 publication mode is enabled and
  frame names (``map``, ``odom``, ``base_link``) are coherent.

See also
--------

* :ref:`mola_lo_architecture`
* :ref:`mola_lo_state_estimators`
* :ref:`mola_lo_localization_plugins`
