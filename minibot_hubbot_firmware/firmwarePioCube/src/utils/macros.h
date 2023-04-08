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

#define CONCAT_STR_LITERAL_HELPER(x, y) x y
// e.g. CONCAT_STR_LITERAL("foo", "bar") -> "foobar"
#define CONCAT_STR_LITERAL(x, y) CONCAT_STR_LITERAL_HELPER(x, y)
#define STR_HELPER(x) (#x)
#define STR(x) STR_HELPER(x)