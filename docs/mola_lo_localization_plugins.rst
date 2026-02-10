.. _mola_lo_localization_plugins:

=======================================================
Creating localization/relocalization plugins for MOLA-LO
=======================================================

This guide explains how to add a new plugin module that provides
localization/relocalization capabilities in a MOLA-LO deployment.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

When you need a plugin
----------------------

Create a plugin when you want to:

* Inject a custom global localization source (e.g., map matching,
  visual relocalization, GNSS-map alignment).
* React to runtime re-initialization requests coming from ROS 2
  (``/initialpose`` by default).
* Provide alternative localization hypotheses to be fused with the
  state estimator.

How relocalization requests enter the system
--------------------------------------------

In the default ROS 2 integration, ``BridgeROS2`` maps
``geometry_msgs/PoseWithCovarianceStamped`` messages from
``relocalize_from_topic`` to relocalization calls for all modules
implementing ``mola::Relocalization``.

This means your plugin must implement MOLA relocalization interfaces and run
inside the same MOLA module container as ``BridgeROS2``.

Default topic and behavior are in:
``mola-cli-launchs/lidar_odometry_ros2.yaml``.

Recommended plugin module contract
----------------------------------

At minimum, implement these capabilities in your new module:

1. **Executable module lifecycle**
   (initialize, periodic work, and observation callbacks).
2. **Relocalization interface support** so bridge-triggered requests can call
   into your module.
3. **Localization output publication** using MOLA localization observations so
   downstream consumers (bridge, state estimator) can use your results.

The exact C++ interfaces live in MOLA core packages. In this repository you can
see how modules are configured and selected by class type string
(e.g., ``mola::LidarOdometry`` or ``mola::state_estimation_simple::StateEstimationSimple``).

Integration steps
-----------------

1. Implement the new module in your package and ensure its shared library is in
   the runtime library path.
2. Add it to the MOLA launch YAML under ``modules:`` with a unique ``name``.
3. Set module ``type`` to your fully qualified class name.
4. Set ``raw_data_source`` if your plugin consumes bridge observations.
5. Configure bridge publication filters so ROS 2 publishes either your plugin,
   LO, or state estimator output as needed.

Example module stanza
---------------------

.. code-block:: yaml

   - name: my_relocalizer
     type: my_namespace::MyRelocalizer
     verbosity_level: "INFO"
     raw_data_source: "ros2_bridge"
     params:
       # plugin-specific settings here
       map_file: "/path/to/map.mm"

How to wire outputs to ROS 2
----------------------------

Use these bridge parameters:

* ``publish_tf_from_slam_source``
* ``publish_odometry_msgs_from_slam_source``

Set them to the module name you want to expose (for example,
``my_relocalizer`` or ``state_estimation``).

Validation checklist
--------------------

* Your module appears in MOLA startup logs.
* Sending a pose on ``/initialpose`` triggers your module callback.
* A localization output from your module is visible in bridge-published
  ``/tf`` or odometry (when selected).
* If used with LO, ensure consistent frame IDs (``map``, ``odom``,
  ``base_link``) with REP-105 mode expectations.

See also
--------

* :ref:`mola_lo_architecture`
* :ref:`mola_lo_state_estimators`
