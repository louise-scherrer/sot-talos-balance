/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayols
 */


#include <sot/talos_balance/ft-calibration.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>


#include <dynamic-graph/all-commands.h>
#include <sot/talos_balance/utils/stop-watch.hh>
#include <sot/talos_balance/utils/statistics.hh>

#define CALIB_ITER_TIME 1000 //Iteration needed for sampling and averaging the FT sensors while calibrating

using namespace sot::talos_balance;

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dg::sot::talos_balance;

#define INPUT_SIGNALS  m_right_foot_force_inSIN   << m_left_foot_force_inSIN
#define OUTPUT_SIGNALS m_right_foot_force_outSOUT << m_left_foot_force_outSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef FtCalibration EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FtCalibration,
                                         "FtCalibration");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      FtCalibration::
      FtCalibration(const std::string& name)
        : Entity(name)
        ,m_robot_util(RefVoidRobotUtil())
        ,m_initSucceeded(false)  
        , CONSTRUCT_SIGNAL_IN(right_foot_force_in,  dynamicgraph::Vector)
        , CONSTRUCT_SIGNAL_IN(left_foot_force_in,   dynamicgraph::Vector)
        , CONSTRUCT_SIGNAL_OUT(right_foot_force_out, dynamicgraph::Vector, m_right_foot_force_inSIN)
        , CONSTRUCT_SIGNAL_OUT(left_foot_force_out,  dynamicgraph::Vector, m_left_foot_force_inSIN)
      {

        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS);

        /* Commands. */
        addCommand("init",
                   makeCommandVoid1(*this, &FtCalibration::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Robot reference (string)")));
 
        addCommand("setRightFootWeight",
                   makeCommandVoid1(*this,&FtCalibration::setRightFootWeight,
                                    docCommandVoid1("Set the weight of the right foot underneath the sensor",
                                                    "Vector of default forces in Newton")));
        addCommand("setLeftFootWeight",
                   makeCommandVoid1(*this,&FtCalibration::setLeftFootWeight,
                            docCommandVoid1("Set the weight of the left foot underneath the sensor",
                                            "Vector of default forces in Newton")));

      }

      void FtCalibration::init(const std::string &robotRef)
      {
        std::string localName(robotRef);
        m_initSucceeded = true;
        if (!isNameInRobotUtil(localName))
        {
            m_robot_util = createRobotUtil(localName);
        }
        else
        {
            m_robot_util = getRobotUtil(localName);
        }
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(right_foot_force_out,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
          return s;
        }
        const Vector & right_foot_force = m_right_foot_force_inSIN(iter);
        assert(right_foot_force_in.size()==6  && "Unexpected size of signal right_foot_force_in");
        
        //do offset calibration if needed
        if (m_right_calibration_iter>0)
        {
			m_right_FT_offset_calibration_sum+=right_foot_force;
			m_right_calibration_iter--;
		}
		else
		{
			m_right_FT_offset = m_right_FT_offset_calibration_sum / CALIB_ITER_TIME ; //todo copy
		}
		
		//remove offset
		s = right_foot_force - m_right_FT_offset;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(left_foot_force_out,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal sum before initialization!");
          return s;
        }
        const Vector & left_foot_force = m_left_foot_force_inSIN(iter);
        assert(left_foot_force_in.size()==6  && "Unexpected size of signal left_foot_force_in");
        
        //do offset calibration if needed
        if (m_left_calibration_iter>0)
        {
			m_left_FT_offset_calibration_sum+=left_foot_force;
			m_left_calibration_iter--;
		}
		else
		{
			m_left_FT_offset = m_left_FT_offset_calibration_sum / CALIB_ITER_TIME ; //todo copy
		}
		
		//remove offset
		s = left_foot_force - m_left_FT_offset;
        return s;
      }
      /* --- COMMANDS ---------------------------------------------------------- */

      void FtCalibration::setRightFootWeight(const dynamicgraph::Vector &rightW)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set right foot weight before initialization!");
          return;
        }
        m_right_foot_weight = rightW;
      }

      void FtCalibration::setleftFootWeight(const dynamicgraph::Vector &leftW)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot set left foot weight before initialization!");
          return;
        }
        m_left_foot_weight = leftW;
      }
      
      void FtCalibration::calibrateFeetSensor()
      {
		SEND_WARNING_STREAM_MSG("Sampling FT sensor for offeset calibration... Robot should be in the air, with horizontal feet.");
        m_right_calibration_iter = CALIB_ITER_TIME;
        m_left_calibration_iter = CALIB_ITER_TIME;
        m_right_FT_offset_calibration_sum = 0; // Todo
        m_left_FT_offset_calibration_sum  = 0; // Todo
      }
      
      void ParameterServer::displayRobotUtil()
      {
		SEND_MSG("The specified joint name does not exist: "+name, MSG_TYPE_ERROR)
        m_robot_util->display(std::cout);
      }

      /* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */

      bool ParameterServer::convertJointNameToJointId(const std::string& name, unsigned int& id)
      {
        // Check if the joint name exists
        pinocchio::Model::JointIndex jid = m_robot_util->get_id_from_name(name);
        if (jid<0)
        {
          SEND_MSG("The specified joint name does not exist: "+name, MSG_TYPE_ERROR);
          std::stringstream ss;
          for(pinocchio::Model::JointIndex it=0; it< m_robot_util->m_nbJoints;it++)
            ss<< m_robot_util->get_name_from_id(it) <<", ";
          SEND_MSG("Possible joint names are: "+ss.str(), MSG_TYPE_INFO);
          return false;
        }
        id = (unsigned int )jid;
        return true;
      }

      bool ParameterServer::isJointInRange(unsigned int id, double q)
      {
        const JointLimits & JL = m_robot_util->
        get_joint_limits_from_id((Index)id);

        double jl= JL.lower;
        if(q<jl)
        {
          SEND_MSG("Desired joint angle "+toString(q)+" is smaller than lower limit: "+toString(jl),MSG_TYPE_ERROR);
          return false;
        }
        double ju = JL.upper;
        if(q>ju)
        {
          SEND_MSG("Desired joint angle "+toString(q)+" is larger than upper limit: "+toString(ju),MSG_TYPE_ERROR);
          return false;
        }
        return true;
      }


      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */


      void ParameterServer::display(std::ostream& os) const
      {
        os << "ParameterServer "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
      
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph
