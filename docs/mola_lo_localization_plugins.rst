.. _mola_lo_localization_plugins:

=======================================================
Creating localization/relocalization plugins for MOLA-LO
=======================================================

This guide explains how to add a custom localization module that can be loaded
into the same MOLA system as LiDAR odometry and ROS 2 bridge modules.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

When to build a plugin
----------------------

Build a plugin when one or more of these are true:

* You need a **global localization source** beyond LO's built-in initialization
  (e.g., map matching from another modality, place recognition, GNSS-map alignment).
* You need **runtime relocalization** behavior that differs from built-in LO policies.
* You need to provide **alternative hypotheses** and let a downstream estimator decide.

Where plugin modules fit in the architecture
--------------------------------------------

Plugins run as peer modules under ``modules:`` in launch YAML, typically with:

* ``raw_data_source: "ros2_bridge"`` if they consume bridge-fed observations.
* Optional direct map file loading in their own parameter block.
* Localization outputs published back to MOLA bus for bridge/estimators.

Relocalization request path
---------------------------

Default ROS 2 integration maps ``relocalize_from_topic`` (``/initialpose`` by default)
from ``geometry_msgs/PoseWithCovarianceStamped`` into relocalization requests
for every module implementing ``mola::Relocalization``.

Practical implication: if your plugin implements relocalization and is loaded in
the system, it will be triggered together with other relocalizable modules.

Minimum module contract
-----------------------

For production use, implement at least:

1. **Lifecycle + execution loop**

   * initialization from YAML,
   * observation handling,
   * periodic work (if applicable).

2. **Relocalization API**

   * process incoming pose priors (mean + covariance),
   * support rejection/failure handling when priors are inconsistent.

3. **Localization output publication**

   * publish estimates in a form consumable by bridge/state estimator,
   * provide covariance and source metadata when possible.

4. **Frame consistency policy**

   * define expected input/output frames,
   * explicitly document map/odom/base_link assumptions.

Integration workflow
--------------------

#. Implement and export your module class in your package shared library.
#. Ensure runtime loader can find the library (environment/package install paths).
#. Add the module stanza under ``modules:``.
#. Configure bridge publication source filters to expose your module if desired.
#. Validate relocalization with ``/initialpose`` and runtime logs.

Example module stanza
---------------------

.. code-block:: yaml

   - name: my_relocalizer
     type: my_namespace::MyRelocalizer
     verbosity_level: "INFO"
     raw_data_source: "ros2_bridge"
     params:
       map_file: "/path/to/map.mm"
       score_threshold: 0.75
       max_hypotheses: 3

ROS publication wiring
----------------------

In ``BridgeROS2.params`` set one or both:

* ``publish_tf_from_slam_source: my_relocalizer``
* ``publish_odometry_msgs_from_slam_source: my_relocalizer``

If you prefer estimator-mediated output, keep bridge source set to estimator,
while estimator fuses your plugin's localization observations.

Recommended failure-handling policies
-------------------------------------

To avoid instability in real robots:

* Gate updates by confidence score / innovation checks.
* Avoid abrupt jumps unless explicitly in relocalization mode.
* Publish covariance inflation during uncertain phases.
* Debounce repeated ``/initialpose`` requests.

Testing strategy
----------------

1. **Static test:** known initial pose and known map; verify convergence.
2. **Perturbed prior test:** random offsets in ``/initialpose``; verify recovery region.
3. **Noisy map/data test:** verify plugin rejects bad matches rather than destabilizing output.
4. **Long-run test:** verify no frame drift/mismatch and no source flapping in bridge output.

Troubleshooting
---------------

* **Plugin loads but no effect:** verify module name/type and that outputs are published.
* **No relocalization callback:** verify ``relocalize_from_topic`` and message type.
* **Output not visible in ROS 2:** verify bridge source filters point to your module.
* **Frame jumps:** verify frame convention and REP-105 mode consistency.

See also
--------

* :ref:`mola_lo_architecture`
* :ref:`mola_lo_state_estimators`
