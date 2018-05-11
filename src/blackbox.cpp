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
 m_stateMutex()
, m_cid{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))}
, m_maxSteering{(commandlineArguments["maxSteering"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxSteering"]))) : (25.0f)}
, m_maxAcceleration{(commandlineArguments["maxAcceleration"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxAcceleration"]))) : (5.0f)}
, m_maxDeceleration{(commandlineArguments["maxDeceleration"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxDeceleration"]))) : (5.0f)}
, m_receiveTimeLimit{(commandlineArguments["receiveTimeLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["receiveTimeLimit"]))) : (0.0001f)}
, m_vx()
, m_vy()
, m_yawRate()
, m_net()
, m_newFrame{true}
, m_directionOK{false}
, m_distanceOK {false}
, m_runOK{true}
, m_directionFrame{}
, m_distanceFrame{}
, m_typeFrame{}
, m_directionFrameBuffer{}
, m_distanceFrameBuffer{}
, m_typeFrameBuffer{}
, m_lastDirectionId{}
, m_lastDistanceId{}
, m_lastTypeId{}
, m_newDirectionId{true}
, m_newDistanceId{true}
, m_newTypeId{true}
, m_directionTimeReceived{}
, m_distanceTimeReceived{}
, m_typeTimeReceived{}
, m_nConesInFrame{}
, m_objectPropertyId{}
, m_directionId{}
, m_distanceId{}
, m_typeId{}
, m_surfaceId{}
{
  m_surfaceId = rand();
  std::cout<<"m_runOK: "<<m_runOK<<std::endl;
  std::cout<<"m_newFrame: "<<m_newFrame<<std::endl;
  std::cout<<"m_directionOK: "<<m_directionOK<<std::endl;
  std::cout<<"m_newDirectionId: "<<m_newDirectionId<<std::endl;
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
  if(a_container.dataType() == opendlv::logic::perception::ObjectProperty::ID()){
      std::cout << "RECIEVED AN OBJECTPROPERY!" << std::endl;
      auto object = cluon::extractMessage<opendlv::logic::perception::ObjectProperty>(std::move(a_container));
      int objectId = object.objectId();
      auto nConesInFrame = object.property();

      if (m_newFrame) { // If true, a frame has just been sent for processing
        m_newFrame = false;
        m_nConesInFrame = std::stoul(nConesInFrame);
        m_objectPropertyId = objectId; // Currently not used.
      }
  }

  if (a_container.dataType() == opendlv::logic::perception::ObjectDirection::ID()) {
      int objectId;
      {
        std::unique_lock<std::mutex> lockDirection(m_directionMutex);
        auto object = cluon::extractMessage<opendlv::logic::perception::ObjectDirection>(std::move(a_container));
        objectId = object.objectId();
        cluon::data::TimeStamp containerStamp = a_container.sampleTimeStamp();
        double timeStamp = containerStamp.microseconds(); // Save timeStamp for sorting purposes;

          if (m_newDirectionId) {
            m_directionId = (objectId!=m_lastDirectionId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
            m_newDirectionId=(m_directionId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
          }

          float angle = object.azimuthAngle(); //Unpack message

          if (objectId == m_directionId) {
            m_directionFrame[timeStamp] = angle;
            m_directionTimeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
          } else if (objectId != m_lastDirectionId){ // If message doesn't belong to current or previous frame.
            m_directionFrameBuffer[timeStamp] = angle; // Place message content coordinates in buffer
          }
      }
      auto wait = std::chrono::system_clock::now(); // Time point now
      std::chrono::duration<double> dur = wait-m_directionTimeReceived; // Duration since last message recieved to m_surfaceFrame
      double duration = (m_directionId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame
      if (duration>m_receiveTimeLimit) { //Only for debug
        std::cout<<"DURATION TIME DIRECTION EXCEEDED"<<std::endl;
      }
      // Run if frame is full or if we have waited to long for the remaining messages
      if ((m_directionFrame.size()==m_nConesInFrame || duration>m_receiveTimeLimit)) { //!m_newFrame && objectId==m_surfaceId &&
        m_directionOK=true;
        std::cout<<m_directionFrame.size()<<" directionFrames to run"<<"\n";
        std::cout<<m_directionFrameBuffer.size()<<" directionFrames in buffer"<<"\n";
        /*std::cout<<"m_directionOK"<<"\n";*/
      }
  }

  else if(a_container.dataType() == opendlv::logic::perception::ObjectDistance::ID()){
    int objectId;
    {
      std::unique_lock<std::mutex> lockDistance(m_distanceMutex);
      auto object = cluon::extractMessage<opendlv::logic::perception::ObjectDistance>(std::move(a_container));
      objectId = object.objectId();
      cluon::data::TimeStamp containerStamp = a_container.sampleTimeStamp();
      double timeStamp = containerStamp.microseconds(); // Save timeStamp for sorting purposes;

      if (m_newDistanceId) {
        m_distanceId = (objectId!=m_lastDistanceId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
        m_newDistanceId=(m_distanceId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
      }

      float distance = object.distance(); //Unpack message

      if (objectId == m_distanceId) {
        m_distanceFrame[timeStamp] = distance;
        m_distanceTimeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
      } else if (objectId != m_lastDistanceId){ // If message doesn't belong to current or previous frame.
        m_distanceFrameBuffer[timeStamp] = distance; // Place message content coordinates in buffer
      }
    }
    auto wait = std::chrono::system_clock::now(); // Time point now
    std::chrono::duration<double> dur = wait-m_distanceTimeReceived; // Duration since last message recieved to m_surfaceFrame
    double duration = (m_distanceId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame
    if (duration>m_receiveTimeLimit) { //Only for debug
      std::cout<<"DURATION TIME DISTANCE EXCEEDED"<<std::endl;
    }
    // Run if frame is full or if we have waited to long for the remaining messages
    if ((m_distanceFrame.size()==m_nConesInFrame || duration>m_receiveTimeLimit)) { //!m_newFrame && objectId==m_surfaceId &&
      m_distanceOK=true;
      std::cout<<m_distanceFrame.size()<<" distanceFrames to run"<<"\n";
      std::cout<<m_distanceFrameBuffer.size()<<" distanceFrames in buffer"<<"\n";
      /*std::cout<<"m_distanceOK"<<"\n";*/
    }
  }

  else if(a_container.dataType() == opendlv::logic::perception::ObjectType::ID()){
    int objectId;
    {
      std::unique_lock<std::mutex> lockType(m_typeMutex);
      auto object = cluon::extractMessage<opendlv::logic::perception::ObjectType>(std::move(a_container));
      objectId = object.objectId();
      cluon::data::TimeStamp containerStamp = a_container.sampleTimeStamp();
      double timeStamp = containerStamp.microseconds(); // Save timeStamp for sorting purposes;

      if (m_newTypeId) {
        m_typeId = (objectId!=m_lastTypeId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
        m_newTypeId=(m_typeId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
      }

      int type = object.type(); //Unpack message

      if (objectId == m_typeId) {
        m_typeFrame[timeStamp] = type;
        m_typeTimeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
      } else if (objectId != m_lastTypeId){ // If message doesn't belong to current or previous frame.
        m_typeFrameBuffer[timeStamp] = type; // Place message content in buffer
      }
    }
    auto wait = std::chrono::system_clock::now(); // Time point now
    std::chrono::duration<double> dur = wait-m_typeTimeReceived; // Duration since last message recieved to m_surfaceFrame
    double duration = (m_typeId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame
    if (duration>m_receiveTimeLimit) { //Only for debug
      std::cout<<"DURATION TIME TYPE EXCEEDED"<<std::endl;
    }
    // Run if frame is full or if we have waited to long for the remaining messages
    if ((m_typeFrame.size()==m_nConesInFrame || duration>m_receiveTimeLimit) && m_runOK) { //!m_newFrame && objectId==m_surfaceId &&
      if (m_directionOK && m_distanceOK) {
        m_runOK = false;
        std::cout<<"m_runOK"<<std::endl;
        std::thread coneCollector(&BlackBox::initializeCollection, this);
        coneCollector.detach();
      }
      std::cout<<m_typeFrame.size()<<" typeFrames to run"<<"\n";
      std::cout<<m_typeFrameBuffer.size()<<" typeFrames in buffer"<<"\n";
    }
  }
  else if(a_container.dataType() == opendlv::sim::KinematicState::ID()){
  //    std::cout << "RECIEVED A KINEMATICSTATE!" << std::endl;
    auto kinematicState = cluon::extractMessage<opendlv::sim::KinematicState>(std::move(a_container));
    {
      std::unique_lock<std::mutex> lockState(m_stateMutex);
      m_vx = kinematicState.vx();
      m_vy = kinematicState.vy();
      m_yawRate = kinematicState.yawRate();
    }
  }
}

/*void BlackBox::nextContainer(cluon::data::Envelope &a_container)
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
      std::unique_lock<std::mutex> lockState(m_stateMutex);
      m_vx = kinematicState.vx();
      m_vy = kinematicState.vy();
      m_yawRate = kinematicState.yawRate();
    }
  }

} // End of nextContainer*/

void BlackBox::initializeCollection(){
  /*std::cout<< "directionFrames in initializeCollection2: "<<m_directionFrame.size()<<"\n";
  std::cout<< "distanceFrames in initializeCollection2: "<<m_distanceFrame.size()<<"\n";
  std::cout<< "typeFrames in initializeCollection2: "<<m_typeFrame.size()<<"\n";*/
  std::map< double, float > directionFrame;
  std::map< double, float > distanceFrame;
  std::map< double, int > typeFrame;

  if (m_directionFrame.size() == m_distanceFrame.size() && m_directionFrame.size() == m_typeFrame.size()) {
    {
      std::unique_lock<std::mutex> lockDirection(m_directionMutex);
      std::unique_lock<std::mutex> lockDistance(m_distanceMutex);
      std::unique_lock<std::mutex> lockType(m_typeMutex);
      m_newFrame = true;
      m_directionOK = false;
      m_distanceOK = false;
      directionFrame = m_directionFrame;
      distanceFrame = m_distanceFrame;
      typeFrame = m_typeFrame;
      m_directionFrame = m_directionFrameBuffer;
      m_distanceFrame = m_distanceFrameBuffer;
      m_typeFrame = m_typeFrameBuffer;
      m_directionFrameBuffer.clear(); // Clear buffer
      m_distanceFrameBuffer.clear(); // Clear buffer
      m_typeFrameBuffer.clear(); // Clear buffer
      m_lastDirectionId = m_directionId; // Update last object id to ignore late messages
      m_lastDistanceId = m_distanceId; // Update last object id to ignore late messages
      m_lastTypeId = m_typeId; // Update last object id to ignore late messages
      m_newDirectionId = true;
      m_newDistanceId = true;
      m_newTypeId = true;
    }
  }
  else {
    {
      std::unique_lock<std::mutex> lockDirection(m_directionMutex);
      std::unique_lock<std::mutex> lockDistance(m_distanceMutex);
      std::unique_lock<std::mutex> lockType(m_typeMutex);
      m_newFrame = true;
      m_directionOK = false;
      m_distanceOK = false;
      m_directionFrame = m_directionFrameBuffer;
      m_distanceFrame = m_distanceFrameBuffer;
      m_typeFrame = m_typeFrameBuffer;
      m_directionFrameBuffer.clear(); // Clear buffer
      m_distanceFrameBuffer.clear(); // Clear buffer
      m_typeFrameBuffer.clear(); // Clear buffer
      m_lastDirectionId = m_directionId; // Update last object id to ignore late messages
      m_lastDistanceId = m_distanceId; // Update last object id to ignore late messages
      m_lastTypeId = m_typeId; // Update last object id to ignore late messages
      m_newDirectionId = true;
      m_newDistanceId = true;
      m_newTypeId = true;
      m_runOK = true;
    }
    return;
  }
  // Unpack
  Eigen::MatrixXd extractedCones(3,directionFrame.size());
  float dir;
  float dis;
  int tpe;
  int I=0;
  for (std::map<double, float >::iterator it = directionFrame.begin();it !=directionFrame.end();it++){
    dir=it->second;
    extractedCones(0,I) = static_cast<double>(dir);
    I++;
  }
  I=0;
  for (std::map<double, float >::iterator it = distanceFrame.begin();it !=distanceFrame.end();it++){
    dis=it->second;
    extractedCones(1,I) = static_cast<double>(dis);
    I++;
  }
  I=0;
  for (std::map<double, int >::iterator it = typeFrame.begin();it !=typeFrame.end();it++){
    tpe=it->second;
    extractedCones(2,I) = static_cast<double>(tpe);
    I++;
  }
  std::cout<<"extractedCones"<<extractedCones<<std::endl;
  int nLeft = 0;
  int nRight = 0;
  int nSmall = 0;
  int nBig = 0;

    for (int i = 0; i < extractedCones.cols(); i++) {
      int type = static_cast<int>(extractedCones(2,i));
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

  //Initialize for next collection
  if(extractedCones.cols() > 0){
    //std::cout << "Extracted Cones " << std::endl;
    //std::cout << extractedCones << std::endl;

    BlackBox::sortIntoSideArrays(extractedCones, nLeft, nRight, nSmall, nBig);
  } // End of if
  m_runOK = true;
} // End of initializeCollection


void BlackBox::sortIntoSideArrays(Eigen::MatrixXd extractedCones, int nLeft, int nRight, int nSmall, int nBig)
{
  int coneNum = extractedCones.cols();
  //Convert to cartesian
  Eigen::MatrixXd cone;
  Eigen::MatrixXd coneLocal = Eigen::MatrixXd::Zero(2,coneNum);

  for(int p = 0; p < coneNum; p++)
  {
    cone = BlackBox::Spherical2Cartesian(extractedCones(0,p), 0.0, extractedCones(1,p));
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
    type = static_cast<int>(extractedCones(2,k));
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
      std::unique_lock<std::mutex> lockState(m_stateMutex);
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

    //Send for Ui TODO: Remove
    opendlv::logic::action::AimPoint o4;
    o4.azimuthAngle(steer);
    o4.distance(2.0f);
    od4.send(o4);

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
