#include "custom-mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include "ns3/string.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("CustomMobilityModel");

NS_OBJECT_ENSURE_REGISTERED(CustomMobilityModel);

TypeId CustomMobilityModel::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::CustomMobilityModel")
        .SetParent<MobilityModel>()
        .SetGroupName("Mobility")
        .AddConstructor<CustomMobilityModel>()
        .AddAttribute("maxHeight",
                      "Change the max height after moving on snake.",
                      DoubleValue(100.0),
                      MakeDoubleAccessor(&CustomMobilityModel::maxHeight),
                      MakeDoubleChecker<double>())
        .AddAttribute("AoI", 
                      "Area of Interest",
                      BoxValue(Box(0.0, 260.0, 0.0, 260.0, 20.0, 80.0)),
                      MakeBoxAccessor(&CustomMobilityModel::AoI),
                      MakeBoxChecker())
        .AddAttribute("Bounds",
                      "Bounds of the area to cruise.",
                      BoxValue(Box(0.0, 260.0, 0.0, 260.0, 0.0, 100.0)),
                      MakeBoxAccessor(&CustomMobilityModel::m_bounds),
                      MakeBoxChecker())
        .AddAttribute("AvgVelocity",
                      "Average velocity of the drone m/s",
                      DoubleValue(5),
                      MakeDoubleAccessor(&CustomMobilityModel::m_avgVelocity),
                      MakeDoubleChecker<double>())
        .AddAttribute("TurnStrenght",
                      "Turn strenght",
                      DoubleValue(50),
                      MakeDoubleAccessor(&CustomMobilityModel::m_turn),
                      MakeDoubleChecker<double>());
    return tid;
}

// Constructor
CustomMobilityModel::CustomMobilityModel() : m_updateInterval(1.0), maxHeight(100.0), m_avgVelocity(5) {}

// Setter for maxHeight
void CustomMobilityModel::SetMaxHeight(double height) {
    maxHeight = height;
}

// Setter for Area of Interest (AoI)
void CustomMobilityModel::SetAoI(Box areaOfInterest) {
    AoI = areaOfInterest;
}

// Setter for Bounds
void CustomMobilityModel::SetBounds(Box bounds) {
    m_bounds = bounds;
}

// Setter for average velocity
void CustomMobilityModel::SetAvgVelocity(double velocity) {
    m_avgVelocity = velocity;
}

// Getters for attributes
double CustomMobilityModel::GetMaxHeight(void) {
    return maxHeight;
}

Box CustomMobilityModel::GetAoI(void) {
    return AoI;
}

Box CustomMobilityModel::GetBounds(void) {
    return m_bounds;
}

double CustomMobilityModel::GetAvgVelocity(void) {
    return m_avgVelocity;
}

void CustomMobilityModel::DoInitialize(void) {
    m_event = Simulator::Schedule(Seconds(m_updateInterval), &CustomMobilityModel::UpdatePosition, this);
    MobilityModel::DoInitialize();
}

void CustomMobilityModel::DoDispose(void) {
    Simulator::Cancel(m_event);
    MobilityModel::DoDispose();
}

Vector CustomMobilityModel::DoGetPosition(void) const {
    return m_position;
}

void CustomMobilityModel::DoSetPosition(const Vector &position) {
    m_position = position;
}

Vector CustomMobilityModel::DoGetVelocity(void) const {
    return m_velocity;
}

int CustomMobilityModel::DoGetState(void) const {
    return m_state;
}

void CustomMobilityModel::setState(int i) {
    m_state = i;
}

int CustomMobilityModel::getState(void) {
    return m_state;
}

bool CustomMobilityModel::getCompState(void) {
    return m_start;
}

std::string CustomMobilityModel::getAoI(void) {
    std::ostringstream oss;
    oss << AoI.xMin << " " << AoI.xMax << " " << AoI.yMin << " " << AoI.yMax << " " << AoI.zMin << " " << AoI.zMax;
    return oss.str();
}

void CustomMobilityModel::UpdatePosition(void) {
  Time delta = Seconds(m_updateInterval);
  old_pos = m_position;
  Vector tmp = Vector(0.0, 0.0, 0.0);
  if (atEight) {
    if (descend == false) { //SNAKE
      if (m_direction) {   // ===>
        tmp.x = m_up.x * m_avgVelocity;
        m_position = m_position + tmp;
      } else {           // <====
        //FIX ALL MOVEMENT
        tmp.x = m_up.x * m_avgVelocity;
        m_position = m_position - tmp;
      }
      if (m_bounds.IsInside(m_position) && m_start) {
        if (AoI.IsInside(m_position)) {
          setState(2);
        } else {
          setState(1);
        }
        m_event = Simulator::Schedule(delta, &CustomMobilityModel::UpdatePosition, this);
        NotifyCourseChange();
      } else {  //HERE
        setState(2);
        //******************************************************************
        //  QUI ADESSO FA DESTRA E SINISTRA E BASTA!!!!
        if (tmp_str <= 0) {
          tmp_str = m_turn;
          std::cout << tmp_str << std::endl;
        }
        //m_position = m_position + m_left;
        m_position.y = m_position.y + (m_left.y * m_avgVelocity); //FIX VECTORS
        tmp_str = tmp_str - m_avgVelocity;
        std::cout << tmp_str << std::endl;
        if (m_direction) {
          m_position = m_position - tmp;
        } else {
          m_position = m_position + tmp;
        }
        if (m_bounds.IsInside(m_position)){
          if (AoI.IsInside(m_position)) {
            setState(2);
          } else {
            setState(1);
          }
          if (tmp_str <= 0) {
            m_start = true;
            if (m_direction == true) {
              m_direction = false;
            } else {
              m_direction = true;
            }
          } else {
            m_start = false;
          }
          m_event = Simulator::Schedule(delta, &CustomMobilityModel::UpdatePosition, this);
          NotifyCourseChange();
        } else {
          //std::cout << "fine -> Y: " << m_position.x << " Y -> " << m_position.y << std::endl;
          descend = true;
          m_position = old_pos;
          m_event = Simulator::Schedule(delta, &CustomMobilityModel::UpdatePosition, this);
          NotifyCourseChange();
        }

        //****************************************************
      }
    } else { //DESCEND
      setState(3);
      tmp.x = up_eight.x * m_avgVelocity;
      tmp.y = up_eight.y * m_avgVelocity;
      tmp.z = up_eight.z * m_avgVelocity;
      if ((m_position.z - tmp.z) > 0){
        m_position.z = m_position.z - tmp.z;
        m_event = Simulator::Schedule(delta, &CustomMobilityModel::UpdatePosition, this);
      } else {
        m_position.z = 0;
        m_event = Simulator::Schedule(delta, &CustomMobilityModel::UpdatePosition, this);
      }
    }
  } else {
    setState(0);
    tmp.x = up_eight.x * m_avgVelocity;
    tmp.y = up_eight.y * m_avgVelocity;
    tmp.z = up_eight.z * m_avgVelocity;
    m_position = m_position + up_eight;
    if ((maxHeight - m_position.z) < tmp.z) {
      //m_position = m_position - tmp;
      m_event = Simulator::Schedule(delta, &CustomMobilityModel::UpdatePosition, this);
      atEight=true;
    } else {
     m_event = Simulator::Schedule(delta, &CustomMobilityModel::UpdatePosition, this); 
    }
  }
}

} // namespace ns3
