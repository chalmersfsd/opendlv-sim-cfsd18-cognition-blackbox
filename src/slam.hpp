/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef SLAM_HPP
#define SLAM_HPP

#include <tuple>
#include <utility>
#include <thread>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include <Eigen/Dense>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "cone.hpp"


class Slam {


private:
 Slam(const Slam &) = delete;
 Slam(Slam &&)      = delete;
 Slam &operator=(const Slam &) = delete;
 Slam &operator=(Slam &&) = delete;
 typedef std::tuple<opendlv::logic::perception::ObjectDirection,opendlv::logic::perception::ObjectDistance,opendlv::logic::perception::ObjectType> ConePackage;
public:
  Slam(std::map<std::string, std::string> commandlineArguments);
  ~Slam() = default;
  void nextContainer(cluon::data::Envelope data);
  std::pair<bool,std::vector<ConePackage>> getCones();
  std::pair<bool,opendlv::logic::sensation::Geolocation> getPose();
  

 private:
  void setUp(std::map<std::string, std::string> commandlineArguments);
  void setupOptimizer();
  void tearDown();
  bool CheckContainer(uint32_t objectId, cluon::data::TimeStamp timeStamp);
  bool isKeyframe(cluon::data::TimeStamp startTime);
  void addOdometryMeasurement(Eigen::Vector3d pose);
  void optimizeGraph();
  void localizer(Eigen::Vector3d pose, Eigen::MatrixXd cones);
  Eigen::Vector3d updatePoseFromGraph();
  Eigen::Vector3d updatePose(Eigen::Vector3d pose, Eigen::Vector2d errorDistance);
  void addPoseToGraph(Eigen::Vector3d pose);
  void performSLAM(Eigen::MatrixXd Cones);
  Eigen::MatrixXd conesToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd Cones);
  Eigen::Vector3d coneToGlobal(Eigen::Vector3d pose, Eigen::MatrixXd Cone);
  Eigen::Vector3d Spherical2Cartesian(double azimuth, double zenimuth, double distance);
  void addConesToMap(Eigen::MatrixXd cones, Eigen::Vector3d pose);
  void addConeMeasurement(Cone cone, Eigen::Vector3d measurement);
  void addConeToGraph(Cone cone, Eigen::Vector3d measurement);
  void initializeCollection();
  bool loopClosing(Cone cone);
  double distanceBetweenCones(Cone c1, Cone c2);
  void updateMap();
  //bool newCone(Eigen::MatrixXd cone,int poseId);



  /*Member variables*/
  g2o::SparseOptimizer m_optimizer;
  int32_t m_timeDiffMilliseconds = 110;
  cluon::data::TimeStamp m_lastTimeStamp;
  Eigen::MatrixXd m_coneCollector;
  uint32_t m_lastObjectId;
  std::mutex m_coneMutex;
  std::mutex m_sensorMutex;
  std::mutex m_mapMutex;
  std::mutex m_optimizerMutex;
  Eigen::Vector3d m_odometryData;
  std::array<double,2> m_gpsReference;
  std::vector<Cone> m_map;
  double m_newConeThreshold= 1;
  cluon::data::TimeStamp m_keyframeTimeStamp;
  double m_timeBetweenKeyframes = 0.5;
  double m_coneMappingThreshold = 67;
  uint32_t m_currentConeIndex = 0;
  int m_poseId = 1000;
  uint32_t m_conesPerPacket = 20;
  bool m_sendConeData = false;
  bool m_sendPoseData = false;
  bool m_newFrame;
  bool m_loopClosing = false;
  bool m_loopClosingComplete = false;
  Eigen::Vector3d m_sendPose;
  std::mutex m_sendMutex;
  

    // Constants for degree transformation
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
  const double PI = 3.14159265f;
};


#endif
