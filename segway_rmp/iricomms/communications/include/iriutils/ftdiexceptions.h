#ifndef FTDI_EXCEPTIONS
#define FTDI_EXCEPTIONS

#include "commexceptions.h"
#include "ftd2xx.h"
#include <string>

/**
 * \brief vector with all the error messages
 *
 * This constant vector has the strings corresponding to all possible errors
 * of any FTDI device. The value of the error can be used to index the table
 * and get the corresponding string.
 */
extern const std::string error_messages[];

/**
 * \brief FTDI exception class
 *
 * This class implements the exceptions for the FTDI communication class. 
 * In addition to the basic error message provided by the base class CException, 
 * this exception class provides also the unique identifier of the communication
 * device that generated the exception.
 *
 * Also, similarly to other exception classes, it appends a class identifer
 * string ("[CFTDI class] - ") to the error message in order to identify 
 * the class that generated the exception.
 *
 * The base class can be used to catch any exception thrown by the application
 * or also, this class can be used in order to catch only exceptions generated 
 * by CFTDI objects.
 *
 */
class CFTDIException : public CCommException
{
  public:
    /**
     * \brief Constructor
     *
     * The constructor calls the base class constructor to add the general
     * exception identifier and then adds the class identifier string 
     * "[CFTDI class]" and the supplied error message. 
     *
     * It also appends the unique identifier of the communication device 
     * that generated the exception. So, the total exception message will 
     * look like this:
     *
     * \verbatim
     * [Exception caught] - <where>
     * Error: [CComm class] - [CFTDI class] - <error message> - <comm id>
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
     *                device unique identifier. This string must be the one used to 
     *                create the object.
     *
     *
     */  
    CFTDIException(const std::string& where,const std::string& error_msg,const std::string& comm_id);
};

/**
 * \brief FTDIServer exception class
 *
 * This class implements the exceptions for the FTDI Server class. 
 * Similarly to other exception classes, it appends a class identifer
 * string ("[CFTDIServer class] - ") to the error message in order to identify 
 * the class that generated the exception.
 *
 * The base class can be used to catch any exception thrown by the application
 * or also, this class can be used in order to catch only exceptions generated 
 * by CFTDIServer objects.
 *
 */
class CFTDIServerException : public CException
{
  public:
    /**
     * \brief Constructor
     *
     * The constructor calls the base class constructor to add the general
     * exception identifier and then adds the class identifier string 
     * "[CFTDIServer class]" and the supplied error message. 
     *
     * It also appends the unique identifier of the communication device 
     * that generated the exception. So, the total exception message will 
     * look like this:
     *
     * \verbatim
     * [Exception caught] - <where>
     * [CComm class] - [CFTDIServer class] - <error message>
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
     */  
    CFTDIServerException(const std::string& where,const std::string& error_msg);
};

#endif
