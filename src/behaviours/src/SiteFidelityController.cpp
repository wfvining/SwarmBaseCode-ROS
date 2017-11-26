#include "SiteFidelityController.h"

#include <cmath>
#include <angles/angles.h>

SiteFidelityController::SiteFidelityController()
{}

SiteFidelityController::~SiteFidelityController()
{}

void SiteFidelityController::Reset()
{
   use_site_fidelity = false;
}

Result SiteFidelityController::DoWork()
{
   Result r;
   r.type = waypoint;
   r.PIDMode = FAST_PID;
   r.wpts.waypoints.push_back(site_fidelity_location);
   r.wristAngle = M_PI/4;
   r.fingerAngle = M_PI/2;
   // our work here is done.
   use_site_fidelity = false;
   returning = true;

   return r;
}

bool SiteFidelityController::ShouldInterrupt()
{
   return use_site_fidelity;
}

bool SiteFidelityController::HasWork()
{
   return use_site_fidelity;
}

void SiteFidelityController::SetSiteFidelityLocation()
{
   site_fidelity_location = current_location;
   // If we want to only use site fidelity stochastically, then we
   // should only set this to true with some probability p.
   use_site_fidelity = true;
}

void SiteFidelityController::SetCurrentLocation(Point c)
{
   current_location = c;
   if(returning
      && hypot(c.x - site_fidelity_location.x, c.y - site_fidelity_location.y) < SITE_FIDELITY_TOLERANCE)
   {
      returning = false;
   }
}

void SiteFidelityController::InterruptedForObstacle()
{
   // XXX: No longer needed?
}
