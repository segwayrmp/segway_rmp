#ifndef _SOCKET_H
#define _SOCKET_H

#include "comm.h"
#include "mutex.h"
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <netdb.h>

/**
 * \brief Socket information structure
 *
 * This structure holds information about the remote IP address and the port 
 * where the socket is connected.  
 *
 */
typedef struct {
  /**
   * \brief address of the socket
   *
   * A string with the desired IP addres to assign to the socket. The IP address 
   * must be passed with the dot format, as shown here
   *
   * * 192.168.0.1
   */
  std::string address;
  /**
   * \brief port of the socket
   *
   * An integer with the port number to be assigned to the socket. This value can 
   * be any positive interger which is not used by any other application.
   */
  int port;
}TSocket_info;

/**
 * \brief Socket driver
 *
 * This class implements a driver to use generic sockets using the TCP/IP protocol.
 * It inherits from CComm to include all the basic functionality of a communication
 * device and reimplements the socket specific functions.
 *
 * This class provides an event which is activated when the connection is remotely
 * closed. This event can be used by the application using the socket to monitor
 * the state iof the connection. To get the identifier of this event, use the 
 * get_connection_closed_event() function.
 *
 * A special constructor is provided to create a new CSocket object from an already
 * initialized socket file descriptor provided by some system function of a 
 * manufacturer library. However, this constructor is no public, and it can only
 * be accessed by the create_socket() function.
 *
 * In case of any error, this class throws an exceptions of the CSocketException class.
 *
 * For more detailed description of the behavior of this class, see the documentation
 * for the CComm base class.
 */

class CSocket : public CComm
{
  private:
    /**
     * \brief Connection closed event identifier
     *
     * This string holds the unique identifier of the event to indicate the
     * closure of a connection. Its value is initialized at construction time
     * with the name assigned to the class to create the unique identifier for
     * the event. Use the get_connection_closed_event() function to get this
     * string.
     */
    std::string connection_closed_event;
    /**
     * \brief cloned socket flag
     *
     * This attribute is used to differentiate between a CSocket object created
     * by the default constructor and those created by the create_socket() function.
     * This flag is mainly used to avoid opening a new socket when the hard_open()
     * function is called. The value of this attribute is changed internally and can 
     * not be accessed from the outside.
     */ 
    bool cloned;
  protected:
    /** 
     * \brief socket file descriptor.
     *
     * This integer holds the file descriptor of the socket associated to the
     * object. This attribute is initialized when either the open() or create_socket()
     * functions are called and can not be modified afterwards. Its value is valid 
     * until the close() function is called.
     *
     */
    int socket_fd;
    /**
     * \brief Constructor with parameters 
     *
     * This constructor is used only by the create_socket() function to create a 
     * new CSocket object and associate it to an already initialized socket file
     * descriptor. Since it is protected, it can not be used as a regular 
     * constructor.
     *
     * \param comm_id a string with the unique identifier of the socket object.
     *                This identifier is used to create the unique identifiers 
     *                of the internal threads and events, so it is imperative
     *                that each socket has a unique name.
     *
     * \param fd a positive integer with the value of the file descriptor of
     *           an already initialized socket. If the file descriptor provided
     *           to this function is not valid or it is not properly initialized,
     *           unexpected errors will ocurr.
     */
    CSocket(const std::string &comm_id,const int fd);
    /**
     * \brief function to create a new socket object from a file descriptor
     *
     * This function is used in a server side application to create a new
     * CSocket object (or any inherited class) from an already initialized
     * socket file descriptor. This is required when the server socket 
     * accepts a new connection with the accept() function, which returns
     * a file descriptor which is not associated to any CSocket object.
     *
     * This function is defined as static so that it can be called without
     * having to create an object of this class, but it can not be accessed
     * outside an object of this class, since it is defined as protected.
     *
     * The most common use of this function is in a server side of a connection
     * just after the accept() function returns with a new client connection, in
     * order to associate the new socket to a CSocket object.
     *
     * \param comm_id a string with the unique identifier of the socket object.
     *                This identifier is used to create the unique identifiers 
     *                of the internal threads and events, so it is imperative
     *                that each socket has a unique name.
     *
     * \param fd a positive integer with the value of the file descriptor of
     *           an already initialized socket. If the file descriptor provided
     *           to this function is not valid or it is not properly initialized,
     *           unexpected errors will ocurr.
     *
     * \return a new CSocket object initialized with the parameters passed as 
     *         arguments to the function.
     */
    static CSocket *create_socket(const std::string &comm_id, const int fd);
     /**
      * \brief Function to actually open the device
      *
      * This function is called automatically when the base class open() function
      * is called. By default, it initializes the socket file descriptor using the
      * socket() system call. But, if the object has been created by the 
      * create_socket() function, this function does nothing.
      *
      * This class does not need any parameter since the socket is created by 
      * default using a TCP/IP protocol and the AF_INET socket family. So the
      * argument passed to this function is ignored and may be NULL.
      * 
      * This function can throw any CSocketException object exception or the generic 
      * CCommException class.
      *
      * \param comm_dev this parameter is ignored in this case and can be set to NULL.
      *                 It is only keeped for backward compatiblity with the CComm
      *                 class.
      */
    virtual void hard_open(void *comm_dev=NULL);
    /**
      * \brief Function to actually configure the device
      * 
      * This function is called automatically when the base class config() function
      * is called. In this case this function does nothing, since all the necessay
      * parameters are provide to the open() function. However, it is necessary to 
      * call the config() function to successfully change the internal state of the
      * communication device.
      *
      * The provided parameter in this case can be NULL since no configuration is
      * needed. This function can throw any CCommException object exception or else 
      * any exception class defined by the inherited class.
      *
      * \param config This parameter is not used since no configuration is required.
      *               It is only keeped for backward compatiblity with the CComm
      *               class.
      */
     virtual void hard_config(void *config=NULL);
    /**
     * \brief Function to actually read from the device
     *
     * This function is automatically called when the new data received event is 
     * activated. The read() function from the base class gets data from the 
     * internal queue, so this function is not used. It must try to read the 
     * ammount of data specified and store it in the data buffer provided without 
     * blocking. Also, it must return the number of bytes actually read from the 
     * devicve, since they may be different than the desired value.
     *
     * In case of any error, this function throws a CSocketException exception.
     *
     * \param data a reference to the buffer where the received data must be 
     * copied. The necessary memory for this buffer must be allocated before 
     * calling this function and have enough size to store all the desired data.
     * If this buffer is not initialized, the function throws an exception.
     *
     * \param len a positive interger that indicates the number of byte to be
     * read from the communication device. This value must be at most the length 
     * of the data buffer provided to the function.
     *
     * \return an integer with the number of bytes actually read. These number 
     * coincide with the desired number if there is enough data in the internal 
     * queue, but it could be smaller if not.
     */
    virtual int hard_read(unsigned char *data, int len);
    /**
     * \brief Hard write function
     *
     * This function is automatically called when the base class write() function
     * is called. It must try to write the desired ammount of data to the communication 
     * device without blocking. Also it must return the number of bytes actually
     * written to the communication device since they may be different from the
     * desired value.
     *
     * In case of any error, this function throws a CSocketException exception.
     *
     * \param data a reference to the buffer with the data must be send to the 
     * device. The necessary memory for this buffer must be allocated before 
     * calling this function and have enough size to store all the desired data.
     * If this buffer is not initialized, the function throws an exception.
     *
     * \param len a positive interger that indicates the number of byte to be
     * written to the communication device. This value must be at most the length 
     * of the data buffer provided to the function.
     *
     * \return an integer with the number of bytes actually written. These number 
     * coincide with the desired number if there is enough data in the internal 
     * queue, but it could be smaller if not.
     */
    virtual int hard_write(unsigned char *data, int len);
    /**
     * \brief Function to actually get the number of bytes availables
     *
     * This function is called when the new data received event is activated.
     * It must get the number of data bytes available from the communication
     * device and return its value without blocking.
     *
     * In case of any error, this function throws a CSocketException exception.
     *
     * \return an integer with the number of bytes available in the receive queue. 
     * This value can be 0 if there is no data available, but there is ni upper
     * limit in its value.
     */
    virtual int hard_get_num_data(void);
    /**
     * \brief Function to actually wait for a given communication event 
     *
     * This function is called in the internal communciation thread to wait for 
     * any event on the communuication device. It must check for any event on the
     * device (reception, end of transmission or error) and return the
     * corresponding identifier. When an event is activated, this function must 
     * return to allow the base class to handle it, and the it is called again
     * to wait for the next event.
     *
     * This function can throw any CCommException object exception or else any
     * exception class defined by the inherited class.
     *
     * \return -1 if there has been any error or else the identifier of the 
     *         communciation event:
     *
     * - 1 for the new data received event
     * - 2 for the error event
     */
    virtual int hard_wait_comm_event(void);
    /** 
     * \brief Hard close function
     *
     * This function is called when the base class close() funciton is called. It
     * must free the device handle initialized by the open() function and also free
     * any other resource allocated by the inherited class.
     *
     * This function can throw any CCommException object exception or else any
     * exception class defined by the inherited class.
     */
    virtual void hard_close(void);
  public:
    /**
     * \brief default constructor
     *
     * This constructor creates a new CSocket object. It does not open a physical 
     * socket, it only allocates the necessary resources to use it.
     *
     * \param comm_id  A null terminated string which identifies the 
     *                 communications device. This string is used to created a
     *                 unique identifier for all the threads and events of the
     *                 class.
     */
    CSocket(const std::string &comm_id);
    /** 
     * \brief Function to get the name of Connection Closed Event
     *  
     * This function is used to retrieve the identifier of the connection closed
     * event. This event is initialized at contruction time, and it can be
     * retrieved at any time. 
     *
     * This function only returns a copy of the internal attribute, so the value
     * returned by this function can be modified without affecting the proper
     * function of the class.
     *
     * \return a string with a copy of the identifier of the connection closed
     *         event. This identifier can be used in the wait_first() or wait_all()
     *         functions of the CEventServer class to wait its activation.
     */
    std::string get_connection_closed_event(void);
    /**
     * \brief Destructor
     *
     * This destructor does nothing. The base class destructor is the one in 
     * charge of freeing all the allocated resources.
     */
    virtual ~CSocket();
};

#endif
