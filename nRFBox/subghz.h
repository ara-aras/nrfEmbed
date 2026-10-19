/* ____________________________
   This software is licensed under the MIT License:
   https://github.com/cifertech/nrfbox
   ________________________________________ */

#ifndef SUBGHZ_H
#define SUBGHZ_H

#include "config.h"

namespace SubGhz {
  void subghzSetup();
  void subghzLoop();
  // Called whenever the main menu is redrawn: detaches interrupts and powers the CC1101 down.
  void subghzStop();
}

#endif // SUBGHZ_H
