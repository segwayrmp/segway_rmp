#ifndef SOCKET_EXCEPTIONS
#define SOCKET_EXCEPTIONS

#include "commexceptions.h"

using namespace std;

/**
 * \brief Socket exception class
 *
 * This class implements the exceptions for the CSocket communication class. 
 * In addition to the basic error message provided by the base class CException, 
 * this exception class provides also the unique identifier of the communication
 * device that generated the exception.
 *
 * Also, similarly to other exception classes, it appends a class identifer
 * string ("[CSocket class] - ") to the error message in order to identify 
 * the class that generated the exception.
 *
 * The base class can be used to catch any exception thrown by the application
 * or also, this class can be used in order to catch only exceptions generated 
 * by CSocket objects.
 *
 */
class CSocketException : public CCommException
{
  public:
    /** 
     * \brief Constructor
     *
     * The constructor calls the base class constructor to add the general
     * exception identifier and then adds the class identifier string 
     * "[CSocket class]" and the supplied error message. 
     *
     * It also appends the unique identifier of the communication device 
     * that generated the exception. So, the total exception message will 
     * look like this:
     *
     * \verbatim
     * [Exception caught] - <where>
     * Error: [CComm class] - [CSocket class] - <error message> - <comm id>
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
    CSocketException(const string& where,const string& error_msg,const string& comm_id);
};

/**
 * \brief No connection exception class
 *
 * This class implements a special exception for the CSocket class and its inherited
 * classes that indicates that the connection has been refused. This event ocurrs
 * when a client side application is trying to connect to a server, and it is still
 * not ready to accept connections.
 *
 * This exception can be caught to prevent the client application to close when the 
 * server in not yet ready, and then used to iterate until it is. If functionality
 * is not desired, this exception can be caught as a simple CSocketException or a
 * basic CException.
 *
 * The error message and the information provided by this exception is the same as
 * the information provided by the CSocketException class.
 *
 */
class CSocketNoConnectionException : public CSocketException
{
  public:
    /**
     * \brief Constructor
     *
     * The constructor calls the base class constructor to add the general
     * exception identifier and then adds the class identifier string 
     * "[CSocket class]" and the supplied error message. 
     *
     * It also appends the unique identifier of the communication device 
     * that generated the exception. So, the total exception message will 
     * look like this:
     *
     * \verbatim
     * [Exception caught] - <where>
     * Error: [CComm class] - [CSocket class] - <error message> - <comm id>
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
    CSocketNoConnectionException(const string& where,const string& error_msg,const string& comm_id);
};

#endif
