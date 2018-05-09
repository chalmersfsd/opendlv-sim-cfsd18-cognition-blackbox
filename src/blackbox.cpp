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

#include <iostream>
#include <cstdlib>
#include "blackbox.hpp"

BlackBox::BlackBox(std::map<std::string, std::string> commandlineArguments) :
 m_lastTimeStamp()
, m_coneCollector()
, m_coneMutex()
, m_stateMutex()
, m_newFrame()
, m_timeDiffMilliseconds()
, m_lastTypeId()
, m_surfaceId()
, m_cid{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))}
, m_maxSteering{(commandlineArguments["maxSteering"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxSteering"]))) : (25.0f)}
, m_maxAcceleration{(commandlineArguments["maxAcceleration"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxAcceleration"]))) : (5.0f)}
, m_maxDeceleration{(commandlineArguments["maxDeceleration"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxDeceleration"]))) : (5.0f)}
, m_vx()
, m_vy()
, m_yawRate()
, m_net()
{
  m_coneCollector = Eigen::MatrixXd::Zero(4,100);
  m_newFrame = true;
  m_timeDiffMilliseconds = 150;
  m_lastTypeId = -1;
  m_surfaceId = rand();
  setUp();
}

BlackBox::~BlackBox()
{
}

void BlackBox::setUp()
{
  std::cout << "Setting up blackbox with maxSteering = " << m_maxSteering << ", maxAcceleration = " << m_maxAcceleration << ", maxDeceleration = " << m_maxDeceleration << std::endl;
  NEAT::Population *pop=0;
  NEAT::Genome *start_genome;
  char curword[20];
  int id;
  int pop_size = 1;

  //Read in the start Genome
  std::string const filename = "cfsdstartgenes";
  std::string const HOME = "/opt/opendlv.data/";
  std::string infile = HOME + filename;

  std::ifstream iFile(infile);
  iFile>>curword;
  iFile>>id;
  std::cout<<"Reading in Genome id "<<id<<std::endl;
  start_genome=new NEAT::Genome(id,iFile);  
  iFile.close();
  pop=new NEAT::Population(start_genome,pop_size);
  pop->verify();

  std::vector<NEAT::Organism*>::iterator curorg;
  for(curorg=(pop->organisms).begin();curorg!=(pop->organisms).end();++curorg) {
    NEAT::Organism *org = *curorg;
    NEAT::Network *net;
    net=org->net;
    m_net = net;
  }
  std::cout << "Network extracted" << std::endl;
} // End of setUp

void BlackBox::tearDown()
{
}


void BlackBox::nextContainer(cluon::data::Envelope &a_container)
{

  if(a_container.dataType() == opendlv::logic::perception::Object::ID()){
    std::cout << "RECIEVED AN OBJECT!" << std::endl;
    m_lastTimeStamp = a_container.sampleTimeStamp();
    auto coneObject = cluon::extractMessage<opendlv::logic::perception::Object>(std::move(a_container));
    int conesInFrame = coneObject.objectId();

    if (m_newFrame){
       m_newFrame = false;
       std::thread coneCollector(&BlackBox::initializeCollection, this, conesInFrame);
       coneCollector.detach();
    }
  }


  if (a_container.dataType() == opendlv::logic::perception::ObjectDirection::ID()) {
    //std::cout << "Recieved Direction" << std::endl;
    //Retrive data and timestamp
    m_lastTimeStamp = a_container.sampleTimeStamp();
    auto coneDirection = cluon::extractMessage<opendlv::logic::perception::ObjectDirection>(std::move(a_container));
    uint32_t objectId = coneDirection.objectId();
    {
      std::unique_lock<std::mutex> lockCone(m_coneMutex);
      m_coneCollector(0,objectId) = coneDirection.azimuthAngle();
      m_coneCollector(1,objectId) = coneDirection.zenithAngle();
    }
  }

  else if(a_container.dataType() == opendlv::logic::perception::ObjectDistance::ID()){

    m_lastTimeStamp = a_container.sampleTimeStamp();
    auto coneDistance = cluon::extractMessage<opendlv::logic::perception::ObjectDistance>(std::move(a_container));
    uint32_t objectId = coneDistance.objectId();
    {
      std::unique_lock<std::mutex> lockCone(m_coneMutex);
      m_coneCollector(2,objectId) = coneDistance.distance();
    }
  }

  else if(a_container.dataType() == opendlv::logic::perception::ObjectType::ID()){

    //std::cout << "Recieved Type" << std::endl;
    m_lastTimeStamp = a_container.sampleTimeStamp();
    auto coneType = cluon::extractMessage<opendlv::logic::perception::ObjectType>(std::move(a_container));
    int objectId = coneType.objectId();
    {
      std::unique_lock<std::mutex> lockCone(m_coneMutex);
      m_lastTypeId = (m_lastTypeId<objectId)?(objectId):(m_lastTypeId);
      auto type = coneType.type();
      m_coneCollector(3,objectId) = type;
    }
  }

  else if(a_container.dataType() == opendlv::sim::KinematicState::ID()){
//    std::cout << "RECIEVED A KINEMATICSTATE!" << std::endl;
    auto kinematicState = cluon::extractMessage<opendlv::sim::KinematicState>(std::move(a_container));
    {
      std::unique_lock<std::mutex> lockCone(m_stateMutex);
      m_vx = kinematicState.vx();
      m_vy = kinematicState.vy();
      m_yawRate = kinematicState.yawRate();
    }
  }

} // End of nextContainer

void BlackBox::initializeCollection(int conesInFrame){

  std::cout<<"Collection initialized "<<std::endl;
  bool sleep = true;

  while(sleep) // Can probably be rewritten nicer
  {
    if (m_lastTypeId >= conesInFrame-1)
std::cout<<"m_lastTypeId "<<m_lastTypeId<<std::endl;
std::cout<<"conesInFrame "<<conesInFrame<<std::endl;
        sleep = false;
  } // End of while

std::cout<<"End of sleep loop "<<std::endl;
  //std::cout << "m_lastTypeId2: " << m_lastTypeId<< std::endl;
  //std::cout << "conesInFrame2: " << conesInFrame<< std::endl;
  Eigen::MatrixXd extractedCones;
  int nLeft = 0;
  int nRight = 0;
  int nSmall = 0;
  int nBig = 0;

  {
    std::unique_lock<std::mutex> lockCone(m_coneMutex);
    extractedCones = m_coneCollector.leftCols(m_lastTypeId+1);
    for (int i = 0; i < extractedCones.cols(); i++) {
      int type = static_cast<int>(extractedCones(3,i));
      if(type == 1){ nLeft++; }
      else if(type == 2){ nRight++; }
      else if(type == 3){ nSmall++; }
      else if(type == 4){ nBig++; }
      else
      {
        std::cout << "WARNING! Object " << i << " has invalid cone type: " << type << std::endl;
      } // End of else
    } // End of for

    std::cout << "members: " << nLeft << " " << nRight << " " << nSmall << " " << nBig << std::endl;
    m_coneCollector = Eigen::MatrixXd::Zero(4,100);
    m_lastTypeId = -1;
    m_newFrame = true;
    std::cout<<"Closing frame "<<std::endl;
  }

  //Initialize for next collection
  if(extractedCones.cols() > 0){
    //std::cout << "Extracted Cones " << std::endl;
    //std::cout << extractedCones << std::endl;

    BlackBox::sortIntoSideArrays(extractedCones, nLeft, nRight, nSmall, nBig);
  } // End of if
} // End of initializeCollection


void BlackBox::sortIntoSideArrays(Eigen::MatrixXd extractedCones, int nLeft, int nRight, int nSmall, int nBig)
{
  int coneNum = extractedCones.cols();
  //Convert to cartesian
  Eigen::MatrixXd cone;
  Eigen::MatrixXd coneLocal = Eigen::MatrixXd::Zero(2,coneNum);

  for(int p = 0; p < coneNum; p++)
  {
    cone = BlackBox::Spherical2Cartesian(extractedCones(0,p), extractedCones(1,p), extractedCones(2,p));
    coneLocal.col(p) = cone;
  } // End of for
//std::cout << "ConeLocal: " << coneLocal.transpose() << std::endl;

  Eigen::MatrixXd coneLeft = Eigen::MatrixXd::Zero(2,nLeft);
  Eigen::MatrixXd coneRight = Eigen::MatrixXd::Zero(2,nRight);
  Eigen::MatrixXd coneSmall = Eigen::MatrixXd::Zero(2,nSmall);
  Eigen::MatrixXd coneBig = Eigen::MatrixXd::Zero(2,nBig);
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  int type;

  for(int k = 0; k < coneNum; k++){
    type = static_cast<int>(extractedCones(3,k));
    if(type == 1)
    {
      coneLeft.col(a) = coneLocal.col(k);
      a++;
    }
    else if(type == 2)
    {
      coneRight.col(b) = coneLocal.col(k);
      b++;
    }
    else if(type == 3)
    {
      coneSmall.col(c) = coneLocal.col(k);
      c++;
    }
    else if(type == 4)
    {
      coneBig.col(d) = coneLocal.col(k);
      d++;
    } // End of else
  } // End of for

  Eigen::MatrixXf coneLeft_f = coneLeft.cast <float> ();
  Eigen::MatrixXf coneRight_f = coneRight.cast <float> ();
  Eigen::ArrayXXf sideLeft = coneLeft_f.transpose().array();
  Eigen::ArrayXXf sideRight = coneRight_f.transpose().array();

  BlackBox::generateSurfaces(sideLeft, sideRight);
} // End of sortIntoSideArrays


void BlackBox::generateSurfaces(Eigen::ArrayXXf sideLeft, Eigen::ArrayXXf sideRight){
std::cout << "sideLeft: " << sideLeft.rows() << std::endl;
std::cout << "sideRight: " << sideRight.rows() << std::endl;
  if(sideLeft.rows()==5 && sideRight.rows()==5){
  
    Eigen::ArrayXXd leftSide = sideLeft.cast <double> ();
    Eigen::ArrayXXd rightSide = sideRight.cast <double> ();

    double in[24];  //Input loading array
    double out1;
    double out2; 
    std::vector<NEAT::NNode*>::iterator out_iter;

    in[0] = 1; // Bias
    {
      std::unique_lock<std::mutex> lockCone(m_stateMutex);
      in[1] = static_cast<double>(m_vx);
      in[2] = static_cast<double>(m_vy);
      in[3] = static_cast<double>(m_yawRate);
    }
    in[4] = leftSide(0,0);
    in[5] = leftSide(0,1);
    in[6] = leftSide(1,0);
    in[7] = leftSide(1,1);
    in[8] = leftSide(2,0);
    in[9] = leftSide(2,1);
    in[10] = leftSide(3,0);
    in[11] = leftSide(3,1);
    in[12] = leftSide(4,0);
    in[13] = leftSide(4,1);
    in[14] = rightSide(0,0);
    in[15] = rightSide(0,1);
    in[16] = rightSide(1,0);
    in[17] = rightSide(1,1);
    in[18] = rightSide(2,0);
    in[19] = rightSide(2,1);
    in[20] = rightSide(3,0);
    in[21] = rightSide(3,1);
    in[22] = rightSide(4,0);
    in[23] = rightSide(4,1);
  
    m_net->load_sensors(in);

    //Activate the net
    //If it loops, exit returning only fitness of 1 step
    if(m_net->activate())
    {
      std::cout << "NET ACTIVATED" << std::endl;
      out_iter=m_net->outputs.begin();
      out1=(*out_iter)->activation;
      ++out_iter;
      out2=(*out_iter)->activation;
    }
    std::cout << "OUTS: " << out1*2-1 << " and " << out2*2-1 << std::endl;

    float maxSteer = m_maxSteering;
    float maxAcc = m_maxAcceleration;
    float maxDec = m_maxDeceleration;

    // Send messages
    cluon::OD4Session od4{m_cid,[](auto){}};

    float steer = maxSteer*3.14159265f/180.0f*(out1*2-1);
    opendlv::proxy::GroundSteeringRequest steerRequest;
    steerRequest.groundSteering(steer);
    od4.send(steerRequest);
    std::cout << "Sent steeringRequest: " << steer << std::endl;

    float acc = (out2*2-1);
    if(acc >= 0)
    {
      acc = maxAcc*acc;
      opendlv::proxy::GroundAccelerationRequest accRequest;
      accRequest.groundAcceleration(acc);
      od4.send(accRequest);
      std::cout << "Sent accelerationRequest: " << acc << std::endl;
    }
    else
    {
      acc = -maxDec*acc;
      opendlv::proxy::GroundDecelerationRequest decRequest;
      decRequest.groundDeceleration(acc);
      od4.send(decRequest);
      std::cout << "Sent decelerationRequest: " << acc << std::endl;
    } 
  }
  else
  {
  std::cout << "Not enough cones to run network" << std::endl;
  } 
} // End of generateSurfaces


// copy from perception-detectcone
Eigen::MatrixXd BlackBox::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = Eigen::MatrixXd::Zero(2,1);
  recievedPoint << xData,
                   yData;
  return recievedPoint;
} // End of Spherical2Cartesian




