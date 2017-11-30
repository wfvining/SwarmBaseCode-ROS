#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

 state1 = 1;
 state2 = 0;

}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) 
  {
    attemptCount = 1;

  

    result.type = waypoint;
    Point searchLocation;
    
    TwoPhaseWalk(searchLocation);
   
     
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
   
    return result;
  }

}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}

/*********************************************************/
/*       Two Phase Walk Implementation                   */
/********************************************************/
void SearchController::TwoPhaseWalk(Point SearchLocation)
{
    globalCounter++;
   /* Initiate the first way point to be 30 cm  away from current location*/   
   if (first_waypoint)
   {
      first_waypoint = false;
     
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.3 * cos(searchLocation.theta)); 
      searchLocation.y = currentLocation.y + (0.3 * sin(searchLocation.theta));  
      
      cout << "Transitioning into Phase 1\n";
      
   }

   /* Continue Phase 1 of a two phase walk */
   else
   {      
      if(state1 == 1 && globalCounter != 5 && state2 == 0)
      {
        //select new heading from Gaussian distribution around current heading
        searchLocation.theta = rng->gaussian(currentLocation.theta,1.5708); //90 degrees in radians
        searchLocation.x = currentLocation.x + (0.3 * cos(searchLocation.theta));// 20 cm
        searchLocation.y = currentLocation.y + (0.3 * sin(searchLocation.theta));// 20 cm
        cout << "Rover is in Phase 1\n";    
      }
   

      if(state2 == 1 && globalCounter != 10 && state1 == 0)
      {
        searchLocation.theta = rng->gaussian(currentLocation.theta,1.5708); //90 degrees in radians
        searchLocation.x = currentLocation.x + (0.4 * cos(searchLocation.theta));// 20 cm
        searchLocation.y = currentLocation.y + (0.4 * sin(searchLocation.theta));// 20 cm
        cout << "Rover is in Phase 2\n";
                   
      }

     if(globalCounter == 5)
     {
       state1 = 0;
       state2 = 1;
       cout << "Transitioning into Phase 2\n";
     }

     else if(globalCounter == 10)
     {
       state1 = 1;
       state2 = 0;
       globalCounter = 0;
       cout << "Transitioning into Phase 1\n";
     }
   
   }

}
 


