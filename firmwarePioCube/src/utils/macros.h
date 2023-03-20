#define INIT_FOOTER \
  if (!success) { \
    LOGERROR("FAILED to initialize.");  \
    ROSLOGERROR("FAILED to initialize."); \
  } else { \
    LOGINFO("SUCCESSFULLY initialized."); \
    ROSLOGINFO("SUCCESSFULLY initialized.");  \
  } \
  return success;

#define INIT_HEADER \
  bool success = true; \
  LOGEVENT("Initializing..."); \
  ROSLOGEVENT("Initializing...");
