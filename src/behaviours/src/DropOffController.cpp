#include "DropOffController.h"

DropOffController::DropOffController() {
  rng = new random_numbers::RandomNumberGenerator();
  reachedCollectionPoint = false;

  result.type = behavior;
  result.b = wait;
  result.wristAngle = 0.7;
  result.reset = false;
  interrupt = false;

  circularCenterSearching = false;
  spinner = 0;
  centerApproach = false;
  seenEnoughCenterTags = false;
  prevCount = 0;

  countLeft = 0;
  countRight = 0;

  isPrecisionDriving = false;
  startWaypoint = false;
  timerTimeElapsed = -1;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {

  cout << "Performing DropOffController Work" << endl;
  if(timerTimeElapsed > -1) {
    timerTimeElapsed = (current_time - returnTimer) / 1e3; // Convert from milliseconds to seconds
  }

  //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
  //to resart our search.
  if(reachedCollectionPoint)
  {
    cout << "Reached collection point" << endl;
    if (timerTimeElapsed >= 5)
    {
      if (finalInterrupt)
      {
        result.type = behavior;
        result.b = nextProcess;
        result.reset = true;
        return result;
      }
      else
      {
        finalInterrupt = true;
        cout << "1" << endl;
      }
    }
    else if (timerTimeElapsed >= 0.1)
    {
      isPrecisionDriving = true;
      result.type = precisionDriving;

      result.fingerAngle = M_PI_2; //open fingers
      result.wristAngle = 0; //raise wrist

      result.pd.cmdVel = -0.3;
      result.pd.cmdAngularError = 0.0;
    }

    return result;
  }

  int count = countLeft + countRight;

  if(count <= 0)
  {
    double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);

    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching) {

      result.type = waypoint;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.push_back(this->centerLocation);
      startWaypoint = false;
      isPrecisionDriving = false;

      timerTimeElapsed = 0;

      return result;

    }
    else if (timerTimeElapsed >= 2)//spin search for center
    {
      Point nextSpinPoint;

      //sets a goal that is 60cm from the centerLocation and spinner
      //radians counterclockwise from being purly along the x-axis.
      nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
      nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
      nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

      result.type = waypoint;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.push_back(nextSpinPoint);

      spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
      if (spinner > 2*M_PI) {
        spinner -= 2*M_PI;
      }
      spinSizeIncrease += spinSizeIncrement;
      circularCenterSearching = true;
      //safety flag to prevent us trying to drive back to the
      //center since we have a block with us and the above point is
      //greater than collectionPointVisualDistance from the center.

      returnTimer = current_time;
      timerTimeElapsed = 0;
      return result;
    }  
  }
  

  bool left = (countLeft > 0);
  bool right = (countRight > 0);
  bool centerSeen = (right || left); // basicly that count > 0

  //reset lastCenterTagThresholdTime timout timer to current time
  if ((!centerApproach && !seenEnoughCenterTags) || (centerSeen && !seenEnoughCenterTags)) {
    lastCenterTagThresholdTime = current_time;
  }

  if (centerSeen || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
  {

    cout << "Attempting to reach the center" << endl;
    centerSeen = true;

    if (first_center && isPrecisionDriving)
    {
      first_center = false;
      result.type = behavior;
      result.reset = false;
      result.b = nextProcess;
      return result;
    }
    isPrecisionDriving = true;

    if (seenEnoughCenterTags) //if we have seen enough tags
    {
      if (abs(countLeft - countRight) > 1)
      {
        int offsetFactor = rng->uniformInteger(2, 4);
        if (right && (countLeft - offsetFactor) > countRight) //and there are too many on the left
        {
          right = false; //then we say none on the right to cause us to turn right
        }
        else if (left && (countRight - offsetFactor) > countLeft)
        {
          left = false; //or left in this case
        }
      }
    }

    float turnDirection = rng->uniformReal(1, 3); // Was set to 1, but less randomize it a bit
    //reverse tag rejection when we have seen enough tags that we are on a
    //trajectory in to the square we dont want to follow an edge.
    if (seenEnoughCenterTags)
    {
      turnDirection = -turnDirection;
    }

    result.type = precisionDriving;
    result.wpts.waypoints.clear();

    //otherwise turn till tags on both sides of image then drive straight
    if (left && right) {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }
    else if (right) {
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = -centeringTurnRate * turnDirection;
    }
    else if (left){
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = centeringTurnRate*turnDirection;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }

    //must see greater than this many tags before assuming we are driving into the center and not along an edge.
    if (left && right && count > centerTagThreshold)
    {
      seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
      lastCenterTagThresholdTime = current_time;
    }
    else if (centerSeen) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
    {
      lastCenterTagThresholdTime = current_time;
    }

    centerApproach = true;
    prevCount = count;
    countLeft = 0;
    countRight = 0;
  }

  //was on approach to center and did not seenEnoughCenterTags
  //for lostCenterCutoff seconds so reset.
  else if (centerApproach) {

    float timeSinceSeeingEnoughCenterTags = (current_time - lastCenterTagThresholdTime) /1e3; // Convert from milliseconds to seconds
    if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
    {
      cout << "Lost track of center" << endl;
      //go back to drive to center base location instead of drop off attempt
      reachedCollectionPoint = false;
      seenEnoughCenterTags = false;
      centerApproach = false;

      result.type = waypoint;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.push_back(this->centerLocation);
      if (isPrecisionDriving) {
        result.type = behavior;
        result.b = prevProcess;
        result.reset = false;
      }
      isPrecisionDriving = false;
      interrupt = false;
      precisionInterrupt = false;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }
    return result;
  }

  if(seenEnoughCenterTags)
  {
    if (numPositiveTags == 0 || !centerSeen)
    {
      reachedCollectionPoint = true;
      UpdateCenterLocation();
      centerApproach = false;
      returnTimer = current_time;
    } 
  }

  return result;
}

void DropOffController::UpdateCenterLocation()
{
   centerLocation.x = currentLocation.x;
   centerLocation.y = currentLocation.y;
}

void DropOffController::Reset() {
  result.type = behavior;
  result.b = wait;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError = 0;
  result.fingerAngle = -1;
  result.wristAngle = 0.7;
  result.reset = false;
  result.wpts.waypoints.clear();
  spinner = 0;
  spinSizeIncrease = 0;
  prevCount = 0;
  timerTimeElapsed = -1;

  countLeft = 0;
  countRight = 0;


  //reset flags
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  circularCenterSearching = false;
  isPrecisionDriving = false;
  finalInterrupt = false;
  precisionInterrupt = false;
  targetHeld = false;
  startWaypoint = false;
  first_center = true;
  cout << "6" << endl;

}

void DropOffController::SetTargetData(vector<Tag> tags) {
  countRight = 0;
  countLeft = 0;

  numPositiveTags = 0;

  if(targetHeld) {
    // if a target is detected and we are looking for center tags
    if (tags.size() > 0 && !reachedCollectionPoint) {

      // this loop is to get the number of center tags
      for (int i = 0; i < tags.size(); i++) {
        if (tags[i].getID() == 256) {
           UpdateCenterLocation();

          if(tags[i].calcYaw() >= 0.0)
          {
            numPositiveTags++;
          }
          else
          {
            numNegativeTags++;
          }
          
          // checks if tag is on the right or left side of the image
          if (tags[i].getPositionX() + cameraOffsetCorrection > 0) {
            countRight++;

          } else {
            countLeft++;
          }
        }
      }
      std::cout << "[dropoff] positive tags: " << numPositiveTags << "num tags: " << tags.size();
    }
  }

}

void DropOffController::ProcessData() {
  if((countLeft + countRight) > 0) {
    isPrecisionDriving = true;
  } else {
    startWaypoint = true;
  }
}

bool DropOffController::ShouldInterrupt() {
  ProcessData();
  if (startWaypoint && !interrupt) {
    interrupt = true;
    precisionInterrupt = false;
    return true;
  }
  else if (isPrecisionDriving && !precisionInterrupt) {
    precisionInterrupt = true;
    return true;
  }
  if (finalInterrupt) {
    return true;
  }
}

bool DropOffController::HasWork() {

  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {
    return false;
  }

  return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
  return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
  centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
  currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
  targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
  targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
