#ifndef SYSMON_TYPES_HPP
#define SYSMON_TYPES_HPP

#include <base/time.h>

namespace sysmon {
  struct ExperimentMarker {
    int experimentNr;
    base::Time time;
  };

  struct SystemStatus {
    base::Time time;

    //battery voltage in mV
    double asguardVoltage;

    //joint encoder value [rad]
    double asguardJointEncoder;

    int systemState;
    int packetsPerSec;
    int controlPacketsPerSec;
  };

}


#endif
