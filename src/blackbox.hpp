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

#ifndef OPENDLV_SIM_CFSD18_COGNITION_BLACKBOX_HPP
#define OPENDLV_SIM_CFSD18_COGNITION_BLACKBOX_HPP

#include <opendlv-standard-message-set.hpp>
#include <cluon-complete.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <thread>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <list>
#include <vector>
#include <algorithm>
#include <string>
#include "neat/neat.h"
#include "neat/network.h"
#include "neat/population.h"
#include "neat/organism.h"
#include "neat/genome.h"
#include "neat/species.h"

class BlackBox {
 public:
  BlackBox(std::map<std::string, std::string>);
  BlackBox(BlackBox const &) = delete;
  BlackBox &operator=(BlackBox const &) = delete;
  virtual ~BlackBox();
  virtual void nextContainer(cluon::data::Envelope &);

 private:
  void setUp();
  void tearDown();

  void initializeCollection();
  void sortIntoSideArrays(Eigen::MatrixXd, int, int, int, int);
  void generateSurfaces(Eigen::ArrayXXf, Eigen::ArrayXXf);
  Eigen::MatrixXd Spherical2Cartesian(double, double, double);

  std::mutex m_stateMutex;
  uint16_t m_cid;
  float m_maxSteering;
  float m_maxAcceleration;
  float m_maxDeceleration;
  float m_receiveTimeLimit;
  float m_vx;
  float m_vy;
  float m_yawRate;
  NEAT::Network *m_net;
  bool m_newFrame;
  bool m_directionOK;
  bool m_distanceOK;
  bool m_runOK;
  std::map< double, float > m_directionFrame;
  std::map< double, float > m_distanceFrame;
  std::map< double, int > m_typeFrame;
  std::map< double, float > m_directionFrameBuffer;
  std::map< double, float > m_distanceFrameBuffer;
  std::map< double, int > m_typeFrameBuffer;
  int m_lastDirectionId;
  int m_lastDistanceId;
  int m_lastTypeId;
  bool m_newDirectionId;
  bool m_newDistanceId;
  bool m_newTypeId;
  std::chrono::time_point<std::chrono::system_clock> m_directionTimeReceived;
  std::chrono::time_point<std::chrono::system_clock> m_distanceTimeReceived;
  std::chrono::time_point<std::chrono::system_clock> m_typeTimeReceived;
  uint64_t m_nConesInFrame;
  int m_objectPropertyId;
  int m_directionId;
  int m_distanceId;
  int m_typeId;
  std::mutex m_directionMutex = {};
  std::mutex m_distanceMutex = {};
  std::mutex m_typeMutex = {};
  int m_surfaceId;

  const double DEG2RAD = 0.017453292522222; // PI/180.0

};


#endif
