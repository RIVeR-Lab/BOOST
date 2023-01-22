#ifndef _SRC_VIEW_IVIEW_H_
#define _SRC_VIEW_IVIEW_H_

#include "common.h"

// A virtual class interface for a view.
class IView {
public:
  virtual ~IView();
  virtual bool displayError(const char* errorStr) = 0;
  // Displays the info screen
  virtual bool updateDisplay(State_t mdl) = 0;
  virtual bool initialize() = 0;
};

#endif // _SRC_VIEW_IVIEW_H_