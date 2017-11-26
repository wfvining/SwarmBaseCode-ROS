#ifndef SITE_FIDELITY_CONTROLLER_H
#define SITE_FIDELITY_CONTROLLER_H

#include "Controller.h"

class SiteFidelityController : virtual Controller {
public:
   SiteFidelityController();
   ~SiteFidelityController();

   void Reset() override;
   Result DoWork() override;
   bool ShouldInterrupt() override;
   bool HasWork() override;

   void SetSiteFidelityLocation();
   void SetCurrentLocation(Point c);
   void InterruptedForObstacle();
   bool IsReturning() { return returning; }

protected:
   void ProcessData() override {}

private:
   const double SITE_FIDELITY_TOLERANCE = 0.25;
   Point current_location;
   Point site_fidelity_location;
   bool  use_site_fidelity = false;
   bool  returning = false;
};

#endif // SITE_FIDELITY_CONTROLLER_H