#ifndef _SOCKETCLIENT_H
#define _SOCKETCLIENT_H

#include "socket.h"

/**
 * \brief Client side socket 
 *
 * This class implements the functionality of a client socket. It inherits from
 * the socket class for the basic socket communication issues and adds the
 * necessary resources to work as a client socket.
 *
 * The hard_open() function (called when the base class open() function is called)
 * is extended to try to connect to the desired server. If the server is already 
 * listening, the connection is set. However, if the server is not yet listening,
 * a special exception (CSocketNoConnection) is thrown. This special exception can
 * be catch and used to iterate until the server is ready.
 *
 * The general procedure to work with client sockets is as follows:
 *
 * 1. Create a new CSocketClient object with a unique identifier.
 * 2. Call the open() function with the server IP address and port
 * 3. If a CSocketNoConnection is throw, either repeat step 2 or exit.
 * 4. On success, call the config() function without any parameters
 * 5. Perform any read or write operations on the socket
 * 6. Call the close() function to finish the socket connection.
 *  
 */
class CSocketClient : public CSocket
{
  private:
    /**
     * \brief information on the remote server
     *
     * This structure holds the IP address and port of the remote server to which
     * the client socket is connected. This structure is initialized after a 
     * successfull call to the open() function and can not be modified afterwards,
     * until the socket is closed and reconnected to an other server.
     *
     * By default, the IP address string is empty and the port is initialized to -1.
     * Use the get_remote_port() and get_remote_IP_address() to get the values
     * of this structure at any time.
     */ 
    TSocket_info remote;
    /**
     * \brief Connection flag
     *
     * This flag indicated whether the client socket is connected to a server or 
     * not. By default it is set to false, and it is only set to true after a 
     * successfull call to the open() function. When the socket is closed, and 
     * therefore the connection is severed, this flag is set back to false.
     *
     * Use the is_connected() function to check whether the socket is connected
     * or not.
     */ 
    bool connected;
  protected:
    /**
     * \brief Connect function
     *
     * This function is called when the base class open() function is called, and 
     * it is used to perform the connection to a server. This function requires a 
     * TSocket_info structure as parameter which must be provided to the open() 
     * call. It also calls the hard_open() function of the CSocket class to 
     * initialize the base class attributes.
     *
     * The IP address and port within this structure are used to try to stablish
     * a communication with a remote server. If the server is already listening,
     * the function returns normally with all the internal attributes properly
     * initialized.
     *
     * If the server is not yet listening, this function throws a CSocketNoConnection
     * exception. This exception can be catched and used to iterate the process 
     * until the server is available or terminate the applcation. In case of any
     * other error, this function throws a CSocketException.
     *
     * After the connection is set, it is still necessary to call the config() 
     * function to properly configure the object to send and receive information.
     *
     * \param comm_dev a valid pointer to a TSocket_info structure with the IP
     *                 address and listening port of the desired server. See the
     *                 documentation on the TSocket_info structure for more
     *                 information on the IP address and port format.
     */
    virtual void hard_open(void *comm_dev=NULL);
    /**
     * \brief function to close the client socket
     *
     * This function is called when the close() function of the base class is 
     * called. It calls the hard_close() function of the CSocket class to
     * actually terminate the connection and change all the internal attributes 
     * to reflect that (the connection flag is set back to false and the server
     * IP address and port are set to the default values.
     *
     * In case of any error, this function throws a CSocketException.
     */ 
    virtual void hard_close(void);
  public:
    /** 
     * \brief Constructor
     *
     * This constructor creates a new client socket object with the provided 
     * identifier, but does not connect it to any server yet. To do that, it
     * is necessary to call the open() function of the base class with a 
     * pointer to a TSocket_info structure as a parameter.
     *
     * \param sock_id a null terminated string with the unique identifier for
     *                the socket. This identifier is internally used to create
     *                the unique indentifiers for all threads and events, so 
     *                it is important that a single identifier is not used more
     *                than once.
     */
    CSocketClient(const std::string &sock_id);
    /**
     * \brief function to return the remote server port
     *
     * This function returns the port number of the current connection if the
     * socket is connected to a server. Otherwise it returns -1.
     *
     * \return the server's port number to which it is connected, or -1 if the
     *         socket is not currently connected to any server.
     */ 
    int get_remote_port(void);
    /**
     * \brief function to return the remote server IP address
     *
     * This function returns the IP address of the current connection if the
     * socket is connected to a server. Otherwise it returns an empty string.
     *
     * \return the server'a IP address to which it is connected, or an empty
     *         string if the socket is not currently connected to any server.
     */
    std::string get_remote_IP_address(void); 
    /**
     * \brief function to check whether the socket is connected or not
     *
     * This function checks wether the client socket is connected to a server 
     * or not.
     *
     * \return true is the socket is connected to a server and false otherwise.
     */ 
    bool is_connected(void);
    /**
      * \brief destructor
      *
      * When the object is destroyed, the connection to the server is lost (if
      * it was not closed before) and all the resources of the client are freed. 
      */
    virtual ~CSocketClient();
};

#endif
