FIND_PATH(comm_INCLUDE_DIR comm.h commexceptions.h rs232.h rs232exceptions.h ftdiserver.h ftdimodule.h ftdiexcetpions.h socket.h socketclient.h socketserver.h socketexceptions.h /usr/include/iriutils /usr/local/include/iriutils)


FIND_LIBRARY(comm_LIBRARY
    NAMES comm
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iriutils) 

IF (comm_INCLUDE_DIR AND comm_LIBRARY)
   SET(comm_FOUND TRUE)
ENDIF (comm_INCLUDE_DIR AND comm_LIBRARY)

IF (comm_FOUND)
   IF (NOT comm_FIND_QUIETLY)
      MESSAGE(STATUS "Found comm library: ${comm_LIBRARY}")
   ENDIF (NOT comm_FIND_QUIETLY)
ELSE (comm_FOUND)
   IF (comm_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find comm library")
   ENDIF (comm_FIND_REQUIRED)
ENDIF (comm_FOUND)

