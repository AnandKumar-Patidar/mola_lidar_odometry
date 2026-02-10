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

#include <mola_lidar_odometry/SCQNRelocalizer.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/core/round.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace mola
{

IMPLEMENTS_MRPT_OBJECT(SCQNRelocalizer, FrontEndBase, mola)

SCQNRelocalizer::SCQNRelocalizer() = default;

void SCQNRelocalizer::initialize_frontend(const Yaml & c)
{
  this->setLoggerName("SCQNRelocalizer");

  const auto cfg = c["params"];

  if (cfg.has("scan_topic_sensor_label")) {
    params_.lidar_sensor_label = std::regex(cfg["scan_topic_sensor_label"].as<std::string>());
  }

  YAML_LOAD_OPT(params_, publish_reference_frame, std::string);
  YAML_LOAD_OPT(params_, publish_vehicle_frame, std::string);
  YAML_LOAD_OPT(params_, relocalization_min_score, double);
  YAML_LOAD_OPT(params_, scqn_command_template, std::string);
  YAML_LOAD_OPT(params_, command_working_directory, std::string);

  if (params_.scqn_command_template.empty()) {
    MRPT_LOG_WARN(
      "Parameter 'scqn_command_template' is empty. Plugin will run in fallback mode "
      "(it republishes the relocalization prior as low-confidence estimate).");
  }
}

void SCQNRelocalizer::spinOnce()
{
  std::optional<CachedScan> scan;
  std::optional<PendingReloc> req;
  {
    auto lck = mrpt::lockHelper(data_mtx_);
    if (!pending_request_) return;
    if (!last_scan_) return;
    req = pending_request_;
    scan = last_scan_;
    pending_request_.reset();
  }

  ASSERT_(scan.has_value() && req.has_value());

  mrpt::poses::CPose3DPDFGaussian estPose = req->prior;
  double score = 0.0;

  const bool ok = executeExternalScqn(*scan, *req, estPose, score);

  if (!ok) {
    MRPT_LOG_WARN("SC-QN backend did not provide a valid estimate. Skipping publication.");
    return;
  }

  if (score < params_.relocalization_min_score) {
    MRPT_LOG_WARN_STREAM(
      "SC-QN match rejected due to low score: " << score << " < "
                                                << params_.relocalization_min_score);
    return;
  }

  LocalizationUpdate lu;
  lu.method = "sc_qn";
  lu.reference_frame = params_.publish_reference_frame;
  lu.child_frame = params_.publish_vehicle_frame;
  lu.timestamp = scan->stamp;
  lu.pose = estPose.mean.asTPose();
  lu.cov = estPose.cov;
  lu.quality = score;

  advertiseUpdatedLocalization(lu);

  MRPT_LOG_INFO_STREAM("Published SC-QN relocalization. score=" << score << " pose=" << lu.pose);
}

#if MOLA_VERSION_CHECK(2, 1, 0)
void SCQNRelocalizer::onNewObservation(const CObservation::ConstPtr & o)
#else
void SCQNRelocalizer::onNewObservation(const CObservation::Ptr & o)
#endif
{
  if (!o) return;

  if (
    params_.lidar_sensor_label.has_value() &&
    !std::regex_match(o->sensorLabel, *params_.lidar_sensor_label)) {
    return;
  }

  const auto obs = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(o);
  if (!obs || !obs->pointcloud) return;

  CachedScan s;
  s.stamp = obs->timestamp;

  const auto n = obs->pointcloud->size();
  s.xyz.reserve(3 * n);
  for (size_t i = 0; i < n; i++) {
    float x, y, z;
    obs->pointcloud->getPointFast(i, x, y, z);
    if (!mrpt::isFinite(x) || !mrpt::isFinite(y) || !mrpt::isFinite(z)) continue;
    s.xyz.push_back(x);
    s.xyz.push_back(y);
    s.xyz.push_back(z);
  }

  auto lck = mrpt::lockHelper(data_mtx_);
  last_scan_ = std::move(s);
}

void SCQNRelocalizer::relocalize_near_pose_pdf(const mrpt::poses::CPose3DPDFGaussian & p)
{
  auto lck = mrpt::lockHelper(data_mtx_);
  PendingReloc pr;
  pr.prior = p;
  pr.from_gnss = false;
  pending_request_ = pr;
}

void SCQNRelocalizer::relocalize_from_gnss()
{
  // This plugin relies on external backend and optional prior. Keep request so
  // next scan can be used if backend supports global search.
  auto lck = mrpt::lockHelper(data_mtx_);
  mrpt::poses::CPose3DPDFGaussian broadPrior;
  broadPrior.cov.unit(1.0);
  PendingReloc pr;
  pr.prior = broadPrior;
  pr.from_gnss = true;
  pending_request_ = pr;
}

bool SCQNRelocalizer::executeExternalScqn(
  const CachedScan & scan, const PendingReloc & request, mrpt::poses::CPose3DPDFGaussian & outPose,
  double & outScore)
{
  if (params_.scqn_command_template.empty()) {
    outPose = request.prior;
    outScore = 0.25;
    return true;
  }

  const auto stem = tmpFileStem();
  const std::string scanFil = stem + "_scan.xyz";
  const std::string priorFil = stem + "_prior.txt";
  const std::string resultFil = stem + "_result.txt";

  {
    std::ofstream f(scanFil);
    if (!f.is_open()) {
      MRPT_LOG_ERROR_STREAM("Cannot write scan file: " << scanFil);
      return false;
    }
    for (size_t i = 0; i < scan.xyz.size(); i += 3) {
      f << scan.xyz[i + 0] << ' ' << scan.xyz[i + 1] << ' ' << scan.xyz[i + 2] << '\n';
    }
  }

  {
    std::ofstream f(priorFil);
    if (!f.is_open()) {
      MRPT_LOG_ERROR_STREAM("Cannot write prior file: " << priorFil);
      return false;
    }
    const auto p = request.prior.mean.asTPose();
    f << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.yaw << ' ' << p.pitch << ' ' << p.roll << '\n';
  }

  std::string cmd = params_.scqn_command_template;
  mrpt::system::strReplaceAll(cmd, "{SCAN_FILE}", scanFil);
  mrpt::system::strReplaceAll(cmd, "{PRIOR_FILE}", priorFil);
  mrpt::system::strReplaceAll(cmd, "{RESULT_FILE}", resultFil);

  const std::string runCmd =
    params_.command_working_directory.empty()
      ? cmd
      : mrpt::format("cd %s && %s", params_.command_working_directory.c_str(), cmd.c_str());

  const int ret = std::system(runCmd.c_str());
  if (ret != 0) {
    MRPT_LOG_ERROR_STREAM("SC-QN command failed with code=" << ret << ": " << runCmd);
    return false;
  }

  std::ifstream r(resultFil);
  if (!r.is_open()) {
    MRPT_LOG_ERROR_STREAM("SC-QN result file not found: " << resultFil);
    return false;
  }

  double x = .0, y = .0, z = .0, yaw = .0, pitch = .0, roll = .0;
  double score = .0;
  r >> x >> y >> z >> yaw >> pitch >> roll >> score;
  if (!r.good() && !r.eof()) {
    MRPT_LOG_ERROR_STREAM(
      "Malformed SC-QN result file. Expected: x y z yaw pitch roll score. File: " << resultFil);
    return false;
  }

  outPose = request.prior;
  outPose.mean = mrpt::poses::CPose3D::FromYawPitchRoll(x, y, z, yaw, pitch, roll);
  outScore = score;

  mrpt::system::deleteFile(scanFil);
  mrpt::system::deleteFile(priorFil);
  mrpt::system::deleteFile(resultFil);

  return true;
}

std::string SCQNRelocalizer::tmpFileStem()
{
  return mrpt::format(
    "%s/mola_scqn_%u", mrpt::system::getTempPath().c_str(),
    static_cast<unsigned>(mrpt::round(mrpt::Clock::nowDouble() * 1e6)));
}

}  // namespace mola
