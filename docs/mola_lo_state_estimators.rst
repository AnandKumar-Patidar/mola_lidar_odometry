.. _mola_lo_state_estimators:

============================================
State estimators in MOLA-LO: usage and authoring
============================================

This guide covers how to use built-in estimators with MOLA-LO, how output
selection works in ROS 2, and what to implement for a custom estimator.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

Role of state estimation in MOLA-LO
-----------------------------------

LiDAR odometry provides geometry-driven pose updates. A state estimator sits
alongside LO to:

* smooth and regularize short-term LO noise,
* integrate additional cues (IMU, odom, GNSS, plugin outputs),
* provide predictable covariance and publication timing.

Built-in estimator options
--------------------------

The launch file selects one of:

* ``mola::state_estimation_simple::StateEstimationSimple``
* ``mola::state_estimation_smoother::StateEstimationSmoother``

Selection path:

* launch argument ``use_state_estimator``
* sets env ``MOLA_STATE_ESTIMATOR``
* consumed in ``mola-cli-launchs/lidar_odometry_ros2.yaml`` under module ``type``.

Parameter files and key knobs
-----------------------------

Default config files:

* ``state-estimator-params/state-estimation-simple.yaml``
* ``state-estimator-params/state-estimation-smoother.yaml``

Frequently tuned parameters:

* process noise:
  ``sigma_random_walk_acceleration_linear``,
  ``sigma_random_walk_acceleration_angular``
* motion assumptions:
  ``enforce_planar_motion``
* prior velocity:
  ``initial_twist`` and (smoother) prior sigmas
* smoother-specific temporal behavior:
  ``sliding_window_length``

Practical selection guidance
----------------------------

* Use **simple estimator** when you need low complexity and direct online filtering.
* Use **smoother** when delayed but more stable estimates are acceptable and
  your motion profile benefits from short-window optimization.

Run examples
------------

Use smoother estimator:

.. code-block:: bash

   ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      use_state_estimator:=true \
      state_estimator_config_yaml:=../state-estimator-params/state-estimation-smoother.yaml

Use simple estimator explicitly:

.. code-block:: bash

   ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
      use_state_estimator:=false \
      state_estimator_config_yaml:=../state-estimator-params/state-estimation-simple.yaml

How ROS 2 publication source is chosen
--------------------------------------

Bridge output source can be filtered independently for TF and odometry messages:

* ``MOLA_LOCALIZATION_PUBLISH_TF_SOURCE``
* ``MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE``

These are set in the launch file depending on estimator usage. If your
configuration uses custom module names, ensure these values match the actual
module name used in launch YAML.

Internal cooperation with LO
----------------------------

LO and estimator are instantiated in the same module container so LO can
interoperate with estimator logic. This pattern is explicit in repository tests.

Authoring a new estimator module
--------------------------------

Implementation checklist:

1. Implement MOLA executable module lifecycle:

   * initialize from YAML,
   * process incoming observations,
   * periodic output publication if needed.

2. Implement fusion core:

   * state definition (pose, twist, optional biases),
   * propagation model,
   * update models for each observation type.

3. Publish localization outputs with covariance and source metadata.

4. Expose runtime parameters via YAML:

   * process/update noise,
   * gating thresholds,
   * frame names,
   * publication rate.

5. Export/register class so launch YAML can instantiate by ``type`` string.

Minimal integration snippet
---------------------------

.. code-block:: yaml

   - name: state_estimation
     type: my_namespace::MyStateEstimator
     verbosity_level: "INFO"
     raw_data_source: "ros2_bridge"
     execution_rate: 20
     params: "../state-estimator-params/my-state-estimator.yaml"

Migration strategy for safe rollout
-----------------------------------

1. Start with LO as ROS output source; run estimator in shadow mode.
2. Compare estimator vs LO trajectories and latency.
3. Tune noises/gating with representative datasets.
4. Switch bridge source to estimator once stable.
5. Keep LO source as quick rollback option.

Validation checklist
--------------------

* Estimator receives expected observation streams.
* Covariance is finite and scaled realistically.
* No unstable oscillations under aggressive maneuvers.
* TF and odometry outputs remain frame-consistent over long runs.

See also
--------

* :ref:`mola_lo_architecture`
* :ref:`mola_lo_localization_plugins`
