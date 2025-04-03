#ifndef CUSTOM_MOBILITY_MODEL_H
#define CUSTOM_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/rectangle.h"
#include "ns3/box.h"

#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

namespace ns3 {

class CustomMobilityModel : public MobilityModel {
public:
  static TypeId GetTypeId(void);
  virtual int getState(void);
  virtual bool getCompState(void);
  virtual std::string getAoI(void);
  CustomMobilityModel();
  // Setters for attributes
  virtual void SetMaxHeight(double maxHeight);
  virtual void SetAoI(Box areaOfInterest);
  virtual void SetBounds(Box bounds);
  virtual void SetAvgVelocity(double velocity);

  virtual double GetMaxHeight(void);
  virtual Box GetAoI(void);
  virtual Box GetBounds(void);
  virtual double GetAvgVelocity(void);

private:
  virtual void DoInitialize(void);
  virtual void DoDispose(void);
  virtual Vector DoGetPosition(void) const;
  virtual void DoSetPosition(const Vector &position);
  virtual Vector DoGetVelocity(void) const;
  virtual int DoGetState(void) const;

  void setState(int i);

  void UpdatePosition(void);


  Vector m_position;
  Vector m_velocity;
  
  Vector m_up = Vector(1.0, 0.0, 0.0);
  Vector m_down = Vector(-1.0, 0.0, 0.0);
  Vector m_left = Vector(0.0, 1.0, 0.0);   //VERTICAL (Y)
  Vector m_right = Vector(0.0, -1.0, 0.0);
  Vector up_eight = Vector(0.0, 0.0, 1.0);
  Vector m_vVector = Vector(1.0, 1.0, 1.0); //IN PIU PER ORA NON USATO
  
  
  EventId m_event;                       //!< stored event ID
  Box m_bounds; 
  Box::Side t_left = Box::LEFT;
  Box::Side t_right = Box::RIGHT;
  Box::Side b_left = Box::LEFT;
  Box::Side b_right = Box::RIGHT;
  Box AoI;
  Vector old_pos;
  double m_updateInterval;
  Ptr<UniformRandomVariable> m_random;
  bool m_direction = true;
  bool horizontal = true;
  bool run = true;
  double maxHeight = 50;
  double m_avgVelocity;
  double m_turn;  //STRENGHT
  bool atEight = false;
  bool descend = false;
  int m_state = 0;
  bool m_start = true;
  bool m_directionvert = false;
  double tmp_str = -1;
};

} // namespace ns3

#endif // CUSTOM_MOBILITY_MODEL_H
