/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayols
 */

#include <sot/talos_balance/qualisys-client.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>
#include <sot/talos_balance/utils/statistics.hh>

// #include "RTProtocol.h"
// #include "RTPacket.h"

using namespace sot::talos_balance;

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace dg::sot::talos_balance;

#define INPUT_SIGNALS m_dummySIN
#define OUTPUT_SIGNALS

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef QualisysClient EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(QualisysClient, "QualisysClient");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
QualisysClient::QualisysClient(const std::string& name)
    : Entity(name), CONSTRUCT_SIGNAL_IN(dummy, double), m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS);
  /* Commands. */
  addCommand("init", makeCommandVoid0(*this, &QualisysClient::init, docCommandVoid0("Initialize the entity.")));

  addCommand("registerRigidBody",
             makeCommandVoid1(*this, &QualisysClient::registerRigidBody,
                              docCommandVoid1("Register a rigid body", "Name of the rigid body")));
  addCommand("setMocapIPAdress",
             makeCommandVoid1(*this, &QualisysClient::setMocapIPAdress,
                              docCommandVoid1("Set IP adress of the Mocap server", "IP adress string")));

  addCommand("getRigidBodyList",
             makeCommandVoid0(*this, &QualisysClient::getRigidBodyList,
                              docCommandVoid0("Displays the list of rigid bodies streamed by the mocap server")));
}

void QualisysClient::init() {
  m_initSucceeded = true;
  m_dummySIN = 0;
  // m_thread.join();
  // m_CRTProtocol rtProtocol;
  // const char           m_serverAddr[] = "140.93.6.44";
  // const unsigned short m_basePort = 22222;
  // const int            m_majorVersion = 1;
  // const int            m_minorVersion = 19;
  // const bool           m_bigEndian = false;
  // bool m_dataAvailable = false;
  // bool m_streamFrames = false;
  // unsigned short m_udpPort = 6734;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

/* --- COMMANDS ---------------------------------------------------------- */

void QualisysClient::registerRigidBody(const std::string& RBname) {
  m_RBnames.push_back(RBname);
  int RBidx = m_RBnames.size() - 1;
  Vector7d initRBposition;
  initRBposition << RBidx, 0, 0, 0, 0, 0, 0;
  m_RBpositions.push_back(initRBposition);
  dg::SignalTimeDependent<dg::Vector, int>* sig;
  sig = new dg::SignalTimeDependent<dg::Vector, int>(
      boost::bind(&QualisysClient::readGenericRigidBody, this, RBidx, _1, _2), m_dummySIN,
      getClassName() + "(" + getName() + ")::output(dynamicgraph::Vector)::xyzquat_" + RBname);
  // genericSignalRefs.push_back( sig );
  signalRegistration(*sig);
}

void QualisysClient::setMocapIPAdress(const std::string& ipAdress) { m_serverAddr = ipAdress; }

void QualisysClient::getRigidBodyList() { m_printRigidBodyList = true; }

/* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */

dg::Vector& QualisysClient::readGenericRigidBody(const int RBidx, dg::Vector& res, const int& time) {
  if (res.size() != 7) {
    res.resize(7);
  }
  m_mutex.lock();
  res << m_RBpositions[RBidx];
  m_mutex.unlock();
  return res;
}

void QualisysClient::manageNetworkFrame() {
  CRTProtocol rtProtocol;
  const unsigned short basePort = 22222;
  const int majorVersion = 1;
  const int minorVersion = 19;
  const bool bigEndian = false;
  bool dataAvailable = false;
  bool streamFrames = false;
  unsigned short udpPort = 6734;

  while (true) {
    if (!rtProtocol.Connected()) {
      if (!rtProtocol.Connect(m_serverAddr.c_str(), basePort, &udpPort, majorVersion, minorVersion, bigEndian)) {
        printf("rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
        sleep(1);
        continue;
      }
    }

    if (!dataAvailable) {
      if (!rtProtocol.Read6DOFSettings(dataAvailable)) {
        printf("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
        sleep(1);
        continue;
      }
    }

    if (!streamFrames) {
      if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d)) {
        printf("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
        sleep(1);
        continue;
      }
    }
    CRTPacket::EPacketType packetType;

    if (rtProtocol.ReceiveRTPacket(packetType, true) > 0) {
      if (packetType == CRTPacket::PacketData) {
        float fX, fY, fZ;
        float rotationMatrix[9];

        CRTPacket* rtPacket = rtProtocol.GetRTPacket();

        // printf("Frame %d\n", rtPacket->GetFrameNumber());
        for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++) {
          if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix)) {
            Eigen::Matrix3f rotMat = Eigen::Map<Eigen::Matrix3f>(rotationMatrix);
            Eigen::Quaternionf quat(rotMat);
            const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
            if (m_printRigidBodyList) {
              SEND_MSG(" - " + toString(pTmpStr), MSG_TYPE_INFO);
              std::cout << (" - " + toString(pTmpStr) + "\n");  // Todo remove
            }
            if (pTmpStr) {
              // compare pTmpStr with m_RBnames
              m_mutex.lock();
              std::vector<std::string>::iterator it = std::find(m_RBnames.begin(), m_RBnames.end(), pTmpStr);
              if (it != m_RBnames.end()) {
                int idx = it - m_RBnames.begin();
                m_RBpositions[idx] << fX, fY, fZ, quat.w(), quat.x(), quat.y(), quat.z();
              }
              m_mutex.unlock();
            }
          }
        }
        m_printRigidBodyList = false;
      }
    }

    // boost::this_thread::sleep_for(boost::chrono::seconds{1});
  }
  rtProtocol.StopCapture();
  rtProtocol.Disconnect();
}
/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void QualisysClient::display(std::ostream& os) const {
  os << "QualisysClient " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph
