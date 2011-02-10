#ifndef _SEGWAY_RMP200_EXCEPTIONS
#define _SEGWAY_RMP200_EXCEPTIONS

#include "exceptions.h"

/**
 * \brief Generic segway RMP 200 exception class
 *
 * This class implements the exceptions for the CSegwayRMP200 class. In addition
 * to the basic error message provided by the base class CException, this
 * exception class provides also the unique identifier of the segway robot
 * that generated the exception.
 *
 * Also, similarly to other exception classes, it appends a class identifer
 * string ("[CSegwayRMP200 class] - ") to the error message in order to identify the
 * class that generated the exception.
 *
 * The base class can be used to catch any exception thrown by the application
 * or also, this class can be used in order to catch only exceptions generated 
 * by CComm objects.
 */
class CSegwayRMP200Exception : public CException
{
  public:
    /**
     * \brief Constructor
     *
     * The constructor calls the base class constructor to add the general
     * exception identifier and then adds the class identifier string 
     * "[CSegwayRMP200 class]" and the supplied error message. 
     *
     * It also appends the unique identifier of the segway robot
     * that generated the exception. So, the total exception message will 
     * look like this:
     *
     * \verbatim
     * [Exception caught] - <where>
     * [CSegwayRMP200 class] - <error message> - <segway id>
     * \endverbatim
     *
     * \param where a null terminated string with the information about the name
     *              of the function, the source code filename and the line where
     *              the exception was generated. This string must be generated 
     *              by the _HERE_ macro.
     *
     * \param error_msg a null terminated string that contains the error message.
     *                  This string may have any valid character and there is no
     *                  limit on its length.
     *
     * \param segway_id a null terminated string that contains the segway robot
     *                  unique identifier. This string must be the one used to create
     *                  the communication device.
     *
     */
    CSegwayRMP200Exception(const std::string& where,const std::string& error_msg,const std::string& segway_id);
};


#endif
