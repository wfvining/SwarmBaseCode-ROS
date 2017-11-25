#include "SiteFidelityController.h"

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
   // our work here is done.
   use_site_fidelity = false;

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
}
