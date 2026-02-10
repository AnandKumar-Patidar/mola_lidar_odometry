.. _mola_lo_fastlio_molareloc:

=====================================================================
Using FAST-LIO odometry as source + estimator fusion + MOLA relocalize
=====================================================================

This guide shows a practical integration where FAST-LIO provides high-rate
local odometry while MOLA-LO localizes against a predefined map and a MOLA
state estimator provides the final fused output.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

Integration objective
---------------------

You want all of the following at once:

* Keep FAST-LIO's smooth short-term odometry.
* Use MOLA-LO map-based corrections to constrain drift.
* Publish a fused trajectory through state estimation.

Target architecture
-------------------

1. FAST-LIO publishes TF ``odom->base_link``.
2. Bridge imports this TF as MOLA odometry observations.
3. LO runs with map preload (``.mm`` and optionally ``.simplemap``).
4. State estimator fuses LO+odometry (+optional IMU/GNSS).
5. Bridge publishes estimator-selected TF and/or odometry.

Prerequisites
-------------

* FAST-LIO (or equivalent) is already running and publishing consistent TF.
* Frame names are known and consistent (at least ``odom`` and ``base_link``).
* Predefined map artifacts are available:

  * metric map file ``.mm`` (for local map preload),
  * optional keyframe map ``.simplemap`` (for localization workflows).

Launch recipe
-------------

.. code-block:: bash

   ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      use_state_estimator:=true \
      forward_ros_tf_odom_to_mola:=true \
      mola_tf_estimated_odometry:=odom \
      mola_tf_base_link:=base_link \
      mola_initial_map_mm_file:=/ABS/PATH/prebuilt_map.mm \
      mola_initial_map_sm_file:=/ABS/PATH/prebuilt_keyframes.simplemap

Argument-by-argument explanation
--------------------------------

* ``use_state_estimator:=true``

  * chooses smoother estimator class via launch env wiring,
  * routes default publication source to estimator.

* ``forward_ros_tf_odom_to_mola:=true``

  * enables ``forward_ros_tf_as_mola_odometry_observations`` in bridge,
  * imports FAST-LIO TF odometry into MOLA internal bus.

* ``mola_tf_estimated_odometry:=odom``

  * selects which TF frame pair is queried as odometry source.

* ``mola_initial_map_mm_file`` / ``mola_initial_map_sm_file``

  * map to ``MOLA_LOAD_MM`` and ``MOLA_LOAD_SM``,
  * consumed by LO pipeline to preload map/simplemap.

Configuration interplay to understand
-------------------------------------

There are three distinct localization contributors:

* external odometry (FAST-LIO via TF import),
* LO geometric/map localization,
* state estimator fusion output.

You can inspect each independently by changing bridge source filters:

* ``MOLA_LOCALIZATION_PUBLISH_TF_SOURCE``
* ``MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE``

Set them to LO during debugging, then switch back to estimator for production.

Initial localization and relocalization
---------------------------------------

Startup behavior is controlled by pipeline ``initial_localization.method``
(via ``MOLA_LO_INITIAL_LOCALIZATION_METHOD`` in launch).

Operational recommendations:

* For known start pose, use fixed-pose initialization.
* For unknown starts, rely on your preferred relocalization flow and use
  ``/initialpose`` to inject priors when available.
* After relocalization events, monitor transient behavior before trusting output.

REP-105 and TF publication notes
--------------------------------

If ``publish_localization_following_rep105=true``, published TF chain follows
map->odom->base_link semantics. Ensure this does not conflict with your existing
FAST-LIO TF tree policy.

If your stack already owns some TF edges, choose a single authority per edge to
avoid competing publishers.

Validation procedure
--------------------

1. Confirm FAST-LIO TF is present and stable.
2. Launch MOLA with forward TF odometry enabled.
3. Verify map/simplemap files are loaded at startup.
4. Verify estimator receives and publishes localization outputs.
5. Compare LO-only vs estimator output in RViz/recorded logs.

Troubleshooting
---------------

* **No imported odometry**

  * check FAST-LIO publishes ``odom->base_link``,
  * check ``mola_tf_estimated_odometry`` and base-link names.

* **Map preload not applied**

  * verify absolute paths and file readability,
  * verify selected LO pipeline supports those preload env vars.

* **Unexpected TF jumps**

  * verify direct vs REP-105 mode,
  * ensure one publisher per TF edge,
  * verify estimator/LO source filter selection.

* **Estimator output worse than LO**

  * retune process/update noise,
  * verify observation covariances and frame consistency,
  * reduce aggressiveness of external odometry fusion if needed.

See also
--------

* :ref:`mola_lo_architecture`
* :ref:`mola_lo_state_estimators`
* :ref:`mola_lo_localization_plugins`
