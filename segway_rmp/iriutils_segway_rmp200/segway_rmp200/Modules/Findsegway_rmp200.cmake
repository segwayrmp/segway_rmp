FIND_PATH(segway_RMP200_INCLUDE_DIR segway_rmp200.h segway_rmp200_exceptions.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(segway_RMP200_LIBRARY
    NAMES segway_RMP200
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (segway_RMP200_INCLUDE_DIR AND segway_RMP200_LIBRARY)
   SET(segway_RMP200_FOUND TRUE)
ENDIF (segway_RMP200_INCLUDE_DIR AND segway_RMP200_LIBRARY)

IF (segway_RMP200_FOUND)
   IF (NOT segway_RMP200_FIND_QUIETLY)
      MESSAGE(STATUS "Found Segway RMP200 driver: ${segway_RMP200_LIBRARY}")
   ENDIF (NOT segway_RMP200_FIND_QUIETLY)
ELSE (segway_RMP200_FOUND)
   IF (segway_RMP200_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find segway RMP200 driver")
   ENDIF (segway_RMP200_FIND_REQUIRED)
ENDIF (segway_RMP200_FOUND)

