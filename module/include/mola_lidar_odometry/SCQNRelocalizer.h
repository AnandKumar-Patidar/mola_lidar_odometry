/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

#pragma once

#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/Relocalization.h>
#include <mola_kernel/version.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <mutex>
#include <optional>
#include <regex>
#include <string>
#include <vector>

namespace mola
{
/**
 * Bridge module to integrate external SC-QN relocalization backends.
 *
 * The module buffers the most recent LiDAR point cloud and, upon relocalization
 * requests, can execute an external command that returns a pose hypothesis.
 */
class SCQNRelocalizer : public mola::FrontEndBase,
                        public mola::LocalizationSourceBase,
                        public mola::Relocalization
{
  DEFINE_MRPT_OBJECT(SCQNRelocalizer, mola)

public:
  SCQNRelocalizer();
  ~SCQNRelocalizer() override = default;

  void initialize_frontend(const Yaml & cfg) override;
  void spinOnce() override;
#if MOLA_VERSION_CHECK(2, 1, 0)
  void onNewObservation(const CObservation::ConstPtr & o) override;
#else
  void onNewObservation(const CObservation::Ptr & o) override;
#endif

  void relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian & p) override;
  void relocalize_from_gnss() override;

private:
  struct Parameters
  {
    std::optional<std::regex> lidar_sensor_label;

    std::string publish_reference_frame = "map";
    std::string publish_vehicle_frame = "base_link";

    double relocalization_min_score = 0.2;

    /// If empty, fallback mode is used (republish prior as low-confidence estimate).
    std::string scqn_command_template;

    /// Optional working directory for command execution.
    std::string command_working_directory;
  };

  struct CachedScan
  {
    mrpt::Clock::time_point stamp;
    std::vector<float> xyz;  // x1 y1 z1 x2 y2 z2 ...
  };

  struct PendingReloc
  {
    mrpt::poses::CPose3DPDFGaussian prior;
    bool from_gnss = false;
  };

  Parameters params_;

  std::mutex data_mtx_;
  std::optional<CachedScan> last_scan_;
  std::optional<PendingReloc> pending_request_;

  bool executeExternalScqn(
    const CachedScan & scan, const PendingReloc & request,
    mrpt::poses::CPose3DPDFGaussian & outPose, double & outScore);

  static std::string tmpFileStem();
};

}  // namespace mola
