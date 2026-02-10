.. _mola_lo_scqn_plugin:

=====================================
Using an SC-QN relocalization plugin
=====================================

This page shows how to integrate an **SC-QN** (Scan Context + QN refinement)
localization plugin with MOLA-LO.

This repository now includes ``mola::SCQNRelocalizer`` as a plugin module.
You can also use an external SC-QN backend command (for example from
`FAST-LIO-Localization-SC-QN <https://github.com/engcang/FAST-LIO-Localization-SC-QN>`_).

.. contents::
   :depth: 2
   :local:
   :backlinks: none

Overview
--------

A typical setup uses:

* ``mola::LidarOdometry`` for continuous local motion estimation.
* ``mola::SCQNRelocalizer`` (plugin) for global relocalization hypotheses.
* A state estimator module to fuse/validate both sources.

Minimal launch wiring
---------------------

Use a launch YAML with both modules under ``modules:``:

.. code-block:: yaml

   modules:
     - name: lidar_odometry
       type: mola::LidarOdometry
       params: { }

     - name: scqn_relocalizer
       type: mola::SCQNRelocalizer
       raw_data_source: "ros2_bridge"
       params:
         scan_topic_sensor_label: "lidar"
         relocalization_min_score: 0.22
         scqn_command_template: >-
           ./run_scqn_backend --scan {SCAN_FILE} --prior {PRIOR_FILE} --out {RESULT_FILE}

Runtime plugin loading
----------------------

Run MOLA with the launch file (no extra plugin library is needed when using
this package class):

.. code-block:: bash

   mola-cli -c your_launch.yaml

``scqn_command_template`` placeholders are:

* ``{SCAN_FILE}``: input scan as ASCII ``x y z`` cloud.
* ``{PRIOR_FILE}``: input prior as ``x y z yaw pitch roll``.
* ``{RESULT_FILE}``: output file the backend must write as
  ``x y z yaw pitch roll score``.

ROS 2 bridge integration
------------------------

The bridge can trigger relocalization requests from ``/initialpose`` if the
plugin implements ``mola::Relocalization``. Keep these points in mind:

* Ensure ``relocalize_from_topic`` is enabled in ``BridgeROS2``.
* Ensure the plugin's expected frame convention matches your map/odom frames.
* Publish covariance with each hypothesis/estimate for robust fusion.

Recommended estimator strategy
------------------------------

When fusing LO + SC-QN outputs:

* Trust LO for high-rate local smoothness.
* Use SC-QN to recover from global drift or unknown starts.
* Gate SC-QN corrections by score and innovation thresholds.
* Inflate SC-QN covariance for low-confidence matches.

Troubleshooting
---------------

* **Plugin class not found**: verify module type is exactly ``mola::SCQNRelocalizer``.
* **No relocalization events**: verify bridge ``relocalize_from_topic`` and incoming messages.
* **Frame jumps after corrections**: validate map/odom/base_link frame policy.
* **Frequent false positives**: increase ``relocalization_min_score`` and tighten gating.

See also
--------

* :ref:`mola_lo_localization_plugins`
* :ref:`mola_lo_architecture`
* :ref:`mola_lo_state_estimators`
