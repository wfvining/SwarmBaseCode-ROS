#include "SearchController.h"
#include "DriveController.h"
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

  state = 1;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
  result.set_velocity = false;
  
  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      result.wpts.waypoints.erase(result.wpts.waypoints.begin());
      if(result.wpts.waypoints.empty()) {
        attemptCount = 0;
        site_fidelity = false;
        maxAttempts = 2;
      }
      else
      {
        attemptCount = 1;
      }
    }
    else if(hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) > 1.0) {
      result.set_velocity = true;
      result.velocity = MAX_VELOCITY;
    }
  }

  if (attemptCount > 0 && attemptCount < maxAttempts) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= maxAttempts || attemptCount == 0) 
  {
    attemptCount = 1;

    result.type = waypoint;
       
    TwoPhaseWalk();
        
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
  // don't repeatedly set this.
  if(!succesfullPickup) {
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), currentLocation);
    maxAttempts = 15;
    attemptCount = 1;
    site_fidelity = true;
    // make sure we go in to the search state when we return
    state = 2;
    globalCounter = 0;
  }
  succesfullPickup = true;
}

double SearchController::GetExp(double mean)
{
   double lambda = 1.0/mean;
   double u = rng->uniformReal(0.0,1.0);
   return -log(1.0-u) / lambda;
}

/*********************************************************/
/*       Two Phase Walk Implementation                   */
/********************************************************/
void SearchController::TwoPhaseWalk()
{
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
      if(state == 1)
      {
         double distance = 0.5+GetExp(2.25);
        //select new heading from Gaussian distribution around current heading
         // just go whatever directio we are already faing
         searchLocation.theta = currentLocation.theta + rng->uniformReal(-M_PI/2.0, M_PI/2.0);
        searchLocation.x = currentLocation.x + (distance * cos(searchLocation.theta));// 2 m
        searchLocation.y = currentLocation.y + (distance * sin(searchLocation.theta));// 2 m
        cout << "Rover is in Phase 1\n";
        state = 2;
        globalCounter = 0;
      }
      else if(state == 2)
      {
        searchLocation.theta = rng->gaussian(currentLocation.theta,1.5708); //90 degrees in radians
        searchLocation.x = currentLocation.x + (0.3 * cos(searchLocation.theta));// 20 cm
        searchLocation.y = currentLocation.y + (0.3 * sin(searchLocation.theta));// 20 cm
        cout << "Rover is in Phase 2 [" << globalCounter << "]: (" << searchLocation.x << "," << searchLocation.y << ")" << std::endl;;
        globalCounter++;
      }

      if(globalCounter == 5)
      {
         state = 1;
         cout << "Transitioning into Phase 1\n";
      }
   }

}
 


