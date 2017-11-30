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
}

void SearchController::Reset() {
  result.reset = false;
}

void SearchController::AddWaypoint(Point wpt) {
  if (this->searchingCluster || this->site_fidelity)
  {
    return;
  }
  result.wpts.waypoints.push_back(wpt);
  this->searchingCluster = true;
  cout << "Cluster is " << hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) << " meters away" << endl;
  cout << "Added Waypoint to cluster" << endl;
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
        this->searchingCluster = false;
        attemptCount = 0;
        site_fidelity = false;
        maxAttempts = 5;
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
    Point searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }
    this->searchingCluster = false;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }

}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  for(auto it = result.wpts.waypoints.begin(); it != result.wpts.waypoints.end(); it++)
  {
    it->x -= diffX;
    it->y -= diffY;
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
  }
  succesfullPickup = true;
}


