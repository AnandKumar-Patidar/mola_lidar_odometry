.. _mola_lo_architecture:

=============================================
MOLA-LO architecture and data-flow reference
=============================================

This page complements :ref:`mola_lidar_odometry` with a practical, deployment-focused
architecture view for ROS 2 + MOLA systems.

.. contents::
   :depth: 2
   :local:
   :backlinks: none

Scope and design goals
----------------------

In this repository, architecture decisions are organized around four goals:

* **Robust motion tracking** from LiDAR (+optional IMU/GNSS).
* **Map-centric localization** against prebuilt maps or online maps.
* **Pluggable state fusion** so LO output can be smoothed/fused with other sources.
* **ROS 2 interoperability** through a single bridge module for I/O and TF publication.

High-level module architecture
------------------------------

A typical deployment from ``mola-cli-launchs/lidar_odometry_ros2.yaml`` uses:

* ``mola::LidarOdometry`` (module name: ``lidar_odom``): scan processing,
  motion estimation, local map update, relocalization hooks.
* ``mola::state_estimation_*`` (module name: ``state_estimation``): state fusion/filtering.
* ``mola::BridgeROS2`` (module name: ``ros2_bridge``): ROS 2 subscriptions,
  TF queries/publication, odometry/map message publication.
* Optional ``mola::MolaViz`` for runtime visualization.

Functional layers
-----------------

It helps to think in layers:

1. **Sensor ingest layer** (BridgeROS2)

   * Subscribes to LiDAR/IMU/GNSS topics.
   * Optionally queries TF odometry (``odom->base_link``) and converts it to
     MOLA odometry observations.

2. **Odometry/localization layer** (LidarOdometry)

   * Converts raw scans into internal metric structures.
   * Runs registration (ICP/NDT/GICP pipeline-dependent).
   * Updates local map and emits localization outputs.
   * Handles startup initialization and relocalization requests.

3. **State fusion layer** (state estimator)

   * Fuses odometry/localization observations with optional motion cues.
   * Produces filtered states at configurable rate.

4. **Output/API layer** (BridgeROS2)

   * Publishes TF and ``nav_msgs/Odometry`` from selected source module.
   * Supports direct map->base_link or REP-105 map->odom->base_link publication.

End-to-end data flow
--------------------

Typical runtime sequence:

#. ROS 2 messages enter ``BridgeROS2.params.subscribe``.
#. Bridge converts them into MOLA observations and forwards internally.
#. ``lidar_odom`` consumes LiDAR (+optional IMU/GNSS) and estimates pose.
#. ``state_estimation`` consumes available localization/motion cues and filters.
#. Bridge publishes TF/odometry from configured source module.

Relocalization sequence (from RViz ``/initialpose``):

#. User/tool publishes ``geometry_msgs/PoseWithCovarianceStamped``.
#. Bridge receives it from ``relocalize_from_topic``.
#. Bridge forwards relocalization requests to modules implementing ``mola::Relocalization``.
#. LO/plugin/estimator reacts according to its own policy and publishes updated localization.

Configuration map (what to edit where)
---------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 24 38 38

   * - Objective
     - Primary file/section
     - Typical knobs
   * - Select algorithmic LO behavior
     - ``pipelines/lidar3d-*.yaml``
     - ICP solver/matchers, map update policy, deskew, validity checks
   * - Select state estimator type
     - ``mola-cli-launchs/lidar_odometry_ros2.yaml`` / launch args
     - ``MOLA_STATE_ESTIMATOR`` class string, estimator YAML path
   * - Tune estimator dynamics
     - ``state-estimator-params/*.yaml``
     - process noise, planar constraints, window length, initial twist
   * - ROS topics and frame names
     - ``BridgeROS2.params`` in launch YAML
     - subscribe topics, ``base_link_frame``, ``odom_frame``, ``reference_frame``
   * - TF publication policy
     - ``BridgeROS2.params``
     - ``publish_localization_following_rep105``, source selection filters
   * - Map preload for localization
     - LO pipeline + launch args
     - ``MOLA_LOAD_MM``, ``MOLA_LOAD_SM``

Source selection architecture
-----------------------------

Two independent decisions are often confused:

* **Who estimates pose internally?**
  Controlled by module set (LO only vs LO + estimator/plugins).
* **Who publishes to ROS 2?**
  Controlled by bridge filters:

  * ``publish_tf_from_slam_source``
  * ``publish_odometry_msgs_from_slam_source``

This allows keeping multiple localization producers active while exposing just
one producer externally.

Frame conventions and REP-105 implications
------------------------------------------

Bridge supports:

* **Direct mode** (``publish_localization_following_rep105=false``):
  map/reference frame directly to ``base_link``.
* **REP-105 mode** (``true``): publishes map->odom while odom->base_link is
  obtained from odometry source chain.

For stable integrations, ensure coherence of:

* ``reference_frame`` (typically ``map``),
* ``odom_frame`` (typically ``odom``),
* ``base_link_frame`` (typically ``base_link``).

Common deployment patterns
--------------------------

* **Pure LO**: publish LO output directly.
* **LO + estimator**: LO provides fast geometric updates; estimator smooths and is
  selected as public ROS output source.
* **External odom + LO map localization + estimator**: external odometry imported
  via TF; LO constrains drift with map; estimator outputs fused trajectory.

Validation checklist
--------------------

* Verify expected modules are instantiated at startup.
* Verify sensor topics are subscribed and arriving.
* Verify LO receives correct sensor labels and map preload paths.
* Verify output source filters point to the intended producer.
* Verify TF tree consistency with chosen direct/REP-105 mode.

See also
--------

* :ref:`mola_lo_state_estimators`
* :ref:`mola_lo_localization_plugins`
* :ref:`mola_lo_fastlio_molareloc`
