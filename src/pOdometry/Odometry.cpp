/************************************************************/
/*    NAME: Paolo Marinelli                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Odometry.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

Odometry::Odometry()
{
  m_first_reading = true;
  m_first_x = true;
  m_first_y = true;

  m_current_x = 0.0;
  m_current_y = 0.0;

  m_new_x_values.clear();
  m_new_y_values.clear();

  m_total_distance_meters    = 0.0;
  last_msg_time       = MOOSTime();
  custom_unit         = "None";
  cout << "Odometry constructed" << endl;
}

//---------------------------------------------------------
// Destructor

Odometry::~Odometry()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Odometry::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if (key == "NAV_X") {
      last_msg_time = MOOSTime();
      double x_msg = msg.GetDouble();

      cout << "Received NAV_X: " << x_msg << endl;

      if (m_first_x) {
        m_first_x = false;
        m_current_x = x_msg;
      }
      m_new_x_values.push_back(x_msg);
    }

    else if (key == "NAV_Y")
    {
      last_msg_time = MOOSTime();
      double y_msg = msg.GetDouble();

      cout << "Received NAV_Y: " << y_msg << endl;

      if (m_first_y) {
        m_first_y = false;
        m_current_y = y_msg;
      }
      m_new_y_values.push_back(y_msg);
    }

    else if (key == "RESET_ODOMETRY") {
      bool reset_msg = msg.GetDouble();

      cout << "Received RESET_ODOMETRY: " << reset_msg << endl;
      
      if (reset_msg) {
        m_total_distance_meters = 0.0;
        m_first_reading = true;
        m_first_x = true;
        m_first_y = true;
        m_new_x_values.clear();
        m_new_y_values.clear();
      }
    } 

    else if (key == "SET_CUSTOM_UNIT") {
      custom_unit = getCustomUnit(msg.GetString());
      Notify(pub_odom_unit, custom_unit);
    }

  }
#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Odometry::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Odometry::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Check if we have received both X and Y first values
  if (!m_first_x && !m_first_y) m_first_reading = false;

  if (MOOSTime() - last_msg_time > time_threshold) reportRunWarning("No new NAV_X or NAV_Y messages received recently.");
  

  // If we have first values, process new readings
  if (!m_first_reading) {

    // Check for mismatched number of readings
    if (m_new_x_values.size() != m_new_y_values.size()) {

      reportRunWarning("Mismatched number of X and Y readings. Discarding extra readings.");

      // Determine the minimum size
      size_t min_size = min(m_new_x_values.size(), m_new_y_values.size());
      // Resize both vectors to the minimum size
      m_new_x_values.resize(min_size);
      m_new_y_values.resize(min_size);

    } 
    
    // Process readings if sizes match
    // Iterate through new readings and compute cumulative distance
    for (size_t i = 0; i < m_new_x_values.size(); ++i) {
      double new_x = m_new_x_values[i];
      double new_y = m_new_y_values[i];

      double distance = hypot(new_x - m_current_x, new_y - m_current_y);
      m_total_distance_meters += distance;
      
      // Update current position
      m_current_x = new_x;
      m_current_y = new_y;
    }

    // Clear new readings vectors
    m_new_x_values.clear();
    m_new_y_values.clear();

    // Publish the total distance traveled
    Notify(pub_odom_meters, m_total_distance_meters);

    if (MOOSStrCmp(custom_unit, "feet")) {
      m_total_distance_custom = meters2Feet(m_total_distance_meters);
    }
    else if (MOOSStrCmp(custom_unit, "miles")) {
      m_total_distance_custom = meters2Miles(m_total_distance_meters);
    }
    else if (MOOSStrCmp(custom_unit, "kilometers")) {
      m_total_distance_custom = meters2Kilometers(m_total_distance_meters);
    }

    if (!MOOSStrCmp(custom_unit, "NoNe")) {
      Notify(pub_odom_custom, m_total_distance_custom);
      Notify(pub_odom_unit, custom_unit);
    }

    cout << "Published Distance: " << m_total_distance_meters << " m" << endl;
    cout << "Time threshold: " << time_threshold << " s" << endl;

  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Odometry::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    
    bool handled = false;
    if(param == "time_threshold") {
      time_threshold = getTimeParam(value);
      handled = true;
    } else if (param=="custom_unit"){
      custom_unit = getCustomUnit(value);
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Odometry::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  for (auto p = sub_variables.begin(); p != sub_variables.end(); ++p) {
    Register(*p, 0);
  }
  Register(sub_custom_unit, 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Odometry::buildReport() 
{
  m_msgs << "============================================" << endl;

  ACTable actab(4);

  std::string header = "ODOM  [meters]  |  ODOM  [" + custom_unit + "]|       time_thresh       |       time_from_last_msg        ";
  actab << header;
  actab.addHeaderLines();
  actab << m_total_distance_meters << m_total_distance_custom << time_threshold << MOOSTime() - last_msg_time;
  m_msgs << actab.getFormattedString();

  return(true);
}

// Get the time parameter from the configuration
double Odometry::getTimeParam(const std::string& s) {
 
  if (!isNumber(s)) {
    std::string msg = "Time parameter not a valid number. Using default value of: " + doubleToString(time_threshold, 0) + " seconds.";
    reportConfigWarning(msg);
    return time_threshold;
  }
  double value = atof(s.c_str());
  if (value < eps) {
    std::string msg = "Time parameter cannot be negative. Using default value of: " + doubleToString(time_threshold, 0) + " seconds.";
    reportConfigWarning(msg);
    return time_threshold;
  }
  return value;
}

bool Odometry::isValidUnit(const std::string& s){
  for (auto p = valid_units.begin(); p != valid_units.end(); ++p) {
    if (MOOSStrCmp(s, *p)) return true;
  }
  return false;
}

std::string Odometry::getCustomUnit(const std::string& s) {
  if (!isValidUnit(s)) {
    reportConfigWarning("Invalid custom unit: " + s);
    return custom_unit;
  } else {
    return tolower(s);
  }
}

double Odometry::meters2Feet(double meters) {
  return meters * 3.28084;
}

double Odometry::meters2Kilometers(double meters) {
  return meters * 1e-3;
}

double Odometry::meters2Miles(double meters) {
  return meters * 0.000621371;
}