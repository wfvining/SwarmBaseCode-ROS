#include "DriveController.h"

DriveController::DriveController() {

  fastVelPID.SetConfiguration(fastVelConfig());
  fastYawPID.SetConfiguration(fastYawConfig());

  slowVelPID.SetConfiguration(slowVelConfig());
  slowYawPID.SetConfiguration(slowYawConfig());

  constVelPID.SetConfiguration(constVelConfig());
  constYawPID.SetConfiguration(constYawConfig());


}

DriveController::~DriveController() {}

void DriveController::Reset()
{
  waypoints.clear();

  if (stateMachineState == STATE_MACHINE_ROTATE || stateMachineState == STATE_MACHINE_SKID_STEER)
  {
    stateMachineState = STATE_MACHINE_WAYPOINTS;
  }
}

Result DriveController::DoWork()
{
   searchVelocity = DEFAULT_VELOCITY;

  if(result.type == behavior)
  {
    if(result.b == noChange)
    {
      //if drive controller gets a no change command it is allowed to continue its previouse action
      //normally this will be to follow waypoints but it is not specified as such.
    }

    else if(result.b == wait)
    {
      //do nothing till told otherwise
      left = 0.0;
      right = 0.0;
      stateMachineState = STATE_MACHINE_WAITING;
    }

  }

  else if(result.type == precisionDriving)
  {

    //interpret input result as a precision driving command
    stateMachineState = STATE_MACHINE_PRECISION_DRIVING;

  }

  else if(result.type == waypoint)
  {
    //interpret input result as new waypoints to add into the queue
    ProcessData();

  }

  switch(stateMachineState)
  {

  //Handlers and the final state of STATE_MACHINE are the only parts allowed to call INTERUPT
  //This should be d one as little as possible. I Suggest to Use timeouts to set control bools false.
  //Then only call INTERUPT if bool switches to true.
  case STATE_MACHINE_PRECISION_DRIVING:
  {

    ProcessData();
    break;
  }


  case STATE_MACHINE_WAYPOINTS:
  {

    //Handles route planning and navigation as well as makeing sure all waypoints are valid.

    bool tooClose = true;
    while (!waypoints.empty() && tooClose)
    {
      if (hypot(waypoints.back().x-currentLocation.x, waypoints.back().y-currentLocation.y) < waypointTolerance)
      {
        waypoints.pop_back();
      }
      else
      {
        tooClose = false;
      }
    }
    if (waypoints.empty())
    {
      stateMachineState = STATE_MACHINE_WAITING;
      result.type = behavior;
      interupt = true;
      return result;
    }
    else
    {
      stateMachineState = STATE_MACHINE_ROTATE;
      waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);
      result.pd.setPointYaw = waypoints.back().theta;

      //cout << "**************************************************************************" << endl;
      //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << endl;
      //fall through on purpose
    }
  }

  case STATE_MACHINE_ROTATE:
  {

    // Calculate angle between currentLocation.theta and waypoints.front().theta
    // Rotate left or right depending on sign of angle
    // Stay in this state until angle is minimized

    waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);

    // Calculate the diffrence between current and desired heading in radians.
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);

    //cout << "ROTATE Error yaw:  " << errorYaw << " target heading : " << waypoints.back().theta << " current heading : " << currentLocation.theta << endl;
    //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << " currentLoc x : " << currentLocation.x << " y : " << currentLocation.y << endl;

    result.pd.setPointVel = 0.0;
    //Calculate absolute value of angle

    float abs_error = fabs(angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta));

    // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
    if (abs_error > rotateOnlyAngleTolerance)
    {
      // rotate but dont drive.
      if (result.PIDMode == FAST_PID)
      {
        fastPID(0.0, errorYaw, result.pd.setPointVel, result.pd.setPointYaw);
      }

      break;
    }
    else
    {
      //move to differential drive step
      stateMachineState = STATE_MACHINE_SKID_STEER;

      //fall through on purpose.
    }
  }

  case STATE_MACHINE_SKID_STEER:
  {
      // Calculate angle between currentLocation.x/y and waypoints.back().x/y
      // Drive forward
      // Stay in this state until angle is at least PI/2

    // calculate the distance between current and desired heading in radians
    waypoints.back().theta = atan2(waypoints.back().y - currentLocation.y, waypoints.back().x - currentLocation.x);
    float errorYaw = angles::shortest_angular_distance(currentLocation.theta, waypoints.back().theta);
    float distance = hypot(waypoints.back().x - currentLocation.x, waypoints.back().y - currentLocation.y);

    //cout << "Skid steer, Error yaw:  " << errorYaw << " target heading : " << waypoints.back().theta << " current heading : " << currentLocation.theta << " error distance : " << distance << endl;
    //cout << "Waypoint x : " << waypoints.back().x << " y : " << waypoints.back().y << " currentLoc x : " << currentLocation.x << " y : " << currentLocation.y << endl;

    if(distance >= 1.0)
    {
       searchVelocity = MAX_VELOCITY;
    }

    // goal not yet reached drive while maintaining proper heading.
    if (fabs(errorYaw) < M_PI_2 &&  distance > waypointTolerance)
    {
      // drive and turn simultaniously
      result.pd.setPointVel = searchVelocity;
      if (result.PIDMode == FAST_PID)
      {
        //cout << "linear velocity:  " << linearVelocity << endl;
        fastPID((searchVelocity-linearVelocity) ,errorYaw, result.pd.setPointVel, result.pd.setPointYaw);
      }
    }
    else {
      // stopno change
      left = 0.0;
      right = 0.0;

      // move back to transform step
      stateMachineState = STATE_MACHINE_WAYPOINTS;
    }

    break;
  }

  default:
  {
    break;
  }


  }

  //package data for left and right values into result struct for use in ROSAdapter
  result.pd.right = right;
  result.pd.left = left;

  //return modified struct
  return result;

}

bool DriveController::ShouldInterrupt()
{
  if (interupt)
  {
    interupt = false;
    return true;
  }
  else
  {
    return false;
  }
}

bool DriveController::HasWork() {   }



void DriveController::ProcessData()
{
  if (result.type == waypoint) {
    result.type = behavior;
    result.b = noChange;

    if(result.reset) {
      waypoints.clear();
    }

    if (!result.wpts.waypoints.empty()) {
      waypoints.insert(waypoints.end(),result.wpts.waypoints.begin(), result.wpts.waypoints.end());
      stateMachineState = STATE_MACHINE_WAYPOINTS;
    }
  }
  else if (result.type == precisionDriving)
  {

    if (result.PIDMode == FAST_PID)
    {
      float vel = result.pd.cmdVel -linearVelocity;
      float setVel = result.pd.cmdVel;
      fastPID(vel,result.pd.cmdAngularError, setVel, result.pd.setPointYaw);
    }
    else if (result.PIDMode == SLOW_PID)
    {
      //will take longer to reach the setPoint but has less chanse of an overshoot
      float vel = result.pd.cmdVel -linearVelocity;
      float setVel = result.pd.cmdVel;
      slowPID(vel,result.pd.cmdAngularError, setVel, result.pd.setPointYaw);
    }
    else if (result.PIDMode == CONST_PID)
    {
      float vel = result.pd.cmdVel - linearVelocity;
      float angular = result.pd.cmdAngular - angularVelocity;

      //cout << "Ang. Vel.  " << angularVelocity << "  Ang. Error" << angular << endl;

      constPID(vel, angular ,result.pd.setPointVel, result.pd.setPointYaw);
    }
  }
}


void DriveController::fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw)
{

  // cout << "PID FAST" << endl;

  float velOut = fastVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = fastYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw)
{
  //cout << "PID SLOW" << endl;

  float velOut = slowVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = slowYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw)
{

  //cout << "PID CONST" << endl;

  float velOut = constVelPID.PIDOut(erroVel, setPointVel);
  float yawOut = constYawPID.PIDOut(constAngularError, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}


void DriveController::SetVelocityData(float linearVelocity,float angularVelocity)
{
  this->linearVelocity = linearVelocity;
  this->angularVelocity = angularVelocity;
}




PIDConfig DriveController::fastVelConfig()
{
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 10;
  config.Kd = 2;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::fastYawConfig() {
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 15;
  config.Kd = 5;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/6;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/3;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::slowVelConfig() {
  PIDConfig config;

  config.Kp = 100;
  config.Ki = 8;
  config.Kd = 1.1;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/2;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::slowYawConfig() {
  PIDConfig config;

  config.Kp = 70;
  config.Ki = 16;
  config.Kd = 10;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/6;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constVelConfig() {
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 10;
  config.Kd = 2;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constYawConfig() {
  PIDConfig config;

  config.Kp = 5;
  config.Ki = 5;
  config.Kd = 0;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper;
  config.derivativeAlpha = 0.6;

  return config;

}
