#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  use_ultra_sound = true;
  result.PIDMode = CONST_PID;
}

void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  use_ultra_sound = true;
  delay = current_time;
}

// Avoid crashing into objects detected by the ultraound
void ObstacleController::avoidObstacle()
{
    //obstacle on right side
    if (right < 0.8 || center < 0.8 || left < 0.8)
    {
      result.type = precisionDriving;

      result.pd.cmdAngular = -K_angular;

      result.pd.setPointVel = 0.0;
      result.pd.cmdVel = 0.0;
      result.pd.setPointYaw = 0;
    }
}

// A collection zone was seen in front of the rover and we are not carrying a target
// so avoid running over the collection zone and possibly pushing cubes out.
void ObstacleController::avoidCollectionZone()
{
    //cout << "Avoiding Collection Zone" << endl;

    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    // Decide which side of the rover sees the most april tags and turn away
    // from that side
    if(count_left_collection_zone_tags < count_right_collection_zone_tags) {
      result.pd.cmdAngular = K_angular;
    } else {
      result.pd.cmdAngular = -K_angular;
    }

    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
}


Result ObstacleController::DoWork()
{

  cout << "Obstacle Do Work" << endl;

  clearWaypoints = true;
  set_waypoint = true;
  result.PIDMode = CONST_PID;

  // The obstacle is an april tag marking the collection zone
  if(collection_zone_seen)
  {
    avoidCollectionZone();
  }
  else
  {
    //cout << "Avoiding Obstacle" << endl;
    avoidObstacle();
  }

  if (can_set_waypoint)
  {

    can_set_waypoint = false;
    set_waypoint = false;
    clearWaypoints = false;

    result.type = waypoint;
    result.PIDMode = FAST_PID;
    Point forward;
    forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
    forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(forward);
  }

  return result;
}


void ObstacleController::setSonarData(float sonarleft, float sonarcenter, float sonarright) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;

  if(!use_ultra_sound)
  {
      left = 3.2;
      right = 3.2;
      center = 3.2;
  }

  ProcessData();
}

void ObstacleController::setCurrentLocation(Point currentLocation)
{
  this->currentLocation = currentLocation;
}

void ObstacleController::ProcessData()
{

  //timeout timer for no tag messages
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;

  if (Td >= 0.5)
  {
    collection_zone_seen = false;
    phys= false;

    if (!obstacleAvoided && use_ultra_sound)
    {
      can_set_waypoint = true;
    }

    use_ultra_sound = true;

  }

  //Process sonar info
  if(ignore_center_sonar)
  {
    if(center > reactivate_center_sonar_threshold)
    {
      ignore_center_sonar = false;
    }
    else{
      center = 3;
    }
  }
  else {
    if (center < 0.12) {
      result.wristAngle = 0.7;
    }
    else {
      result.wristAngle = -1;
    }
  }

  if (left < triggerDistance || right < triggerDistance || center < triggerDistance)
  {
    phys = true;
    timeSinceTags = current_time;
  }


  if (collection_zone_seen || phys)
  {
    obstacleDetected = true;
    obstacleAvoided = false;
    can_set_waypoint = false;
  }
  else
  {
    obstacleAvoided = true;
  }
}

// Report April tags seen by the rovers camera so it can avoid
// the collection zone
// Added relative pose information so we know whether the
// top of the AprilTag is pointing towards the rover or away.
// If the top of the tags are away from the rover then treat them as obstacles. 
void ObstacleController::setTagData(vector<Tag> tags)
{
  collection_zone_seen = false;
  count_left_collection_zone_tags = 0;
  count_right_collection_zone_tags = 0;

  string s = "";

  if(targetHeld)
  {
      s = "Has target";
  }
  else
  {
      s = "Doesn't have target";
  }

  //cout << s << endl;

  // this loop is to get the number of center tags
  for (int i = 0; i < tags.size(); i++)
  {
    if (tags[i].getID() == 256)
    {
      if(targetHeld)
      {
          Reset();
          use_ultra_sound = false;
          cout << " MONKEY 1" << endl;
      }
      else
      {
        collection_zone_seen = checkForCollectionZoneTags( tags );
      }

      timeSinceTags = current_time;
    }
  }
}

bool ObstacleController::checkForCollectionZoneTags( vector<Tag> tags )
{

 // cout << "See center tag as obstacle" << endl;

  for ( auto & tag : tags )
  {
  
  //cout << "Yaw Value: " << tag.calcYaw() << endl;

    // Check the orientation of the tag. If we are outside the collection zone the yaw will be positive so treat the collection zone as an obstacle. If the yaw is negative the robot is inside the collection zone and the boundary should not be treated as an obstacle. This allows the robot to leave the collection zone after dropping off a target.
    if ( tag.calcYaw() > 0 ) 
      {
	// checks if tag is on the right or left side of the image
        if (tag.getPositionX() + camera_offset_correction > 0)
        {
	  count_right_collection_zone_tags++;
	  
        }
        else
        {
	  count_left_collection_zone_tags++;
	}
      }
    
  }

  // Did any tags indicate that the robot is inside the collection zone?
  return count_left_collection_zone_tags + count_right_collection_zone_tags > 0;

}

bool ObstacleController::ShouldInterrupt()
{

  if(obstacleDetected && !obstacleInterrupt)
  {
    obstacleInterrupt = true;
    return true;
  }
  else
  {
    if(obstacleAvoided && obstacleDetected)
    {
      Reset();
      return true;
    }
    else
    {
      return false;
    }
  }
}

bool ObstacleController::HasWork()
{
  if (can_set_waypoint && set_waypoint)
  {
    return true;
  }

  return !obstacleAvoided;
}

//ignore center ultrasound
void ObstacleController::setIgnoreCenterSonar()
{
  ignore_center_sonar = true; 
}

void ObstacleController::setCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void ObstacleController::setTargetHeld()
{
  targetHeld = true;

  if (previousTargetState == false)
  {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}
