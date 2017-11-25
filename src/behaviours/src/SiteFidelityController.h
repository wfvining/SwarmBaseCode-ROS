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

protected:
   void ProcessData() override {}

private:
   Point current_location;
   Point site_fidelity_location;
   bool  use_site_fidelity = false;
};

#endif // SITE_FIDELITY_CONTROLLER_H