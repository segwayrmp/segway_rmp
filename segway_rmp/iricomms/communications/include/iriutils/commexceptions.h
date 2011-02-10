#ifndef COMM_EXCEPTIONS
#define COMM_EXCEPTIONS

#include "exceptions.h"

/**
 * \brief Generic communication exception class
 *
 * This class implements the exceptions for the CComm class. In addition
 * to the basic error message provided by the base class CException, this
 * exception class provides also the unique identifier of the communication
 * device that generated the exception.
 *
 * Also, similarly to other exception classes, it appends a class identifer
 * string ("[CComm class] - ") to the error message in order to identify the
 * class that generated the exception.
 *
 * The base class can be used to catch any exception thrown by the application
 * or also, this class can be used in order to catch only exceptions generated 
 * by CComm objects.
 */
class CCommException : public CException
{
  public:
    /**
     * \brief Constructor
     *
     * The constructor calls the base class constructor to add the general
     * exception identifier and then adds the class identifier string 
     * "[CComm class]" and the supplied error message. 
     *
     * It also appends the unique identifier of the communication device
     * that generated the exception. So, the total exception message will 
     * look like this:
     *
     * \verbatim
     * [Exception caught] - <where>
     * [CComm class] - <error message> - <comm id>
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
     * \param comm_id a null terminated string that contains the communication
     *                device unique identifier. This string must be the one used 
     *                to create the communication device.
     *
     */
    CCommException(const std::string& where, const std::string& error_msg,const std::string& comm_id);
};

#endif
