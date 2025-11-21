/************************************************************/
/*    NAME: Paolo Marinelli                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Odometry_HEADER
#define Odometry_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include <string>
#include <vector>

class Odometry : public AppCastingMOOSApp
{
 public:
   Odometry();
   ~Odometry();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
  void    registerVariables();
  double  getTimeParam(const std::string& s);
  double  meters2Kilometers(double meters);
  double  meters2Feet(double meters);
  double  meters2Miles(double meters);
  std::string getCustomUnit(const std::string& s);
  bool    isValidUnit(const std::string& s);


  std::string const         pub_odom_meters     {"ODOMETRY_DIST"};
  std::string const         pub_odom_custom     {"ODOMETRY_CUSTOM_DIST"};
  std::string const         pub_odom_unit       {"ODOMETRY_CUSTOM_UNIT"};
  std::vector<std::string>  sub_variables       {"NAV_X", "NAV_Y", "RESET_ODOMETRY"};
  std::string const         sub_custom_unit     {"SET_CUSTOM_UNIT"};
  std::vector<std::string>  valid_units         {"meters", "kilometers", "feet", "miles"};
  double                    time_threshold      {10.0}; // seconds
  double const              eps                 {1e-6}; 
 private: // Configuration variables

 private: // State variables
  bool m_first_reading;
  bool m_first_x;
  bool m_first_y;

  double m_current_x;
  double m_current_y;

  std::string custom_unit;
  std::vector<double> m_new_x_values;
  std::vector<double> m_new_y_values;

  double last_msg_time;
  double m_total_distance_meters;
  double m_total_distance_custom;
};

#endif 
