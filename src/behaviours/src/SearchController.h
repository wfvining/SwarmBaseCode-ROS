#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();

protected:

  void ProcessData();

private:

  double GetExp(double mean);
  void TwoPhaseWalk();//Two Phase Walk implementation
  
  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;

  Point initialLocation; /* Keep track of starting position */

  int attemptCount = 0;
  int maxAttempts = 2;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
  bool site_fidelity = true;
  /* A fixed value that determines how correlated the direction of the next step is with the  direction of the previous step */

  int state = 1;
  int globalCounter = 0;
   
};

#endif /* SEARCH_CONTROLLER */
