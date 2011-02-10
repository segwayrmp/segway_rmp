#ifndef _SOCKETSERVER_H
#define _SOCKETSERVER_H

#include <queue>
#include "socket.h"

/**
 * \brief possible server states
 *
 * This ennumeration type represents all the possible states of the server, which
 * are used internally to control its flow. The possible states are:
 *
 * * server_created: It is the default state when it is created for the first time. 
 * * server_binded: after successfully calling the open() function, the server is
 *                  attached to an IP address and port.
 * * server_listening: after successfully calling the config() function the server
 *                     is ready to listen to incomming connections. If the server
 *                     is closed, it returns to the server_created state.
 * * server_on: It is the main state when the server is started using the start()
 *              function. In this state, the server is capable of detecting new 
 *              connections and disconnections, as well as send and receive data 
 *              to and from any client. If the server is stopped it returns to the
 *              server_listening state.
 *
 * The next figure shows the server states and the possible transitions between them:
 *
 * \image html server_states.png
 *
 */
typedef enum {server_created, server_binded, server_listening, server_on} server_state;

/**
 * \brief client information
 *
 * This structure holds all the information required by the server side application 
 * to handle a connection with a new client. This structure is initialized when
 * a new connection is stablished, and the get_new_client() function should be used 
 * to retrieve it. Once the connection is closed, the free_client() function should
 * be used to properly free all the allocated resources.
 */
typedef struct 
{
  /**
   * \brief client identifier 
   *
   * This ideintifier is automatically assigned by the server when the connection is
   * stablished. This identifier is unique for each connection and it is used to 
   * identify the client to which send or receive information. 
   */ 
  std::string client_id;
  /**
   * \brief client IP address
   *
   * This string holds the IP address of the client in dot format (192.168.0.1). This 
   * value can be used by the server side application to identify the different 
   * clients connected. This field is purely informative and it is not used for
   * any operation.
   */ 
  std::string IP;
  /**
   * \brief client port
   *  
   * this integer is the port assigned to the client for the connection. This port
   * is automatically assigned by the accept function and can not be modified 
   * afterwards. This filed is purely informative and it is not used by any operation.
   */ 
  int port;
  /**
   * \brief identifier of the disconnect event.
   *
   * This events notifies the server side application that the connection to the
   * client has been closed. This event is not the same as the connection closed
   * event provided by the CSocket class. This event is used internally by the
   * server, which activates this one when the disconnection is detected.
   */ 
  std::string disconnect_event_id;
  /**
   * \brief identifier of the reception event
   *  
   * This event notifies the reception of new data for the corresponding client.
   * This event is the one provided by the CSocket base class and it is provided
   * here because there is no direct access to the CSocket object from outside
   * the class.
   */ 
  std::string rx_event_id;
}TClient_info;

/**
 * \brief Internal client information
 *
 * This structure hold information of the client necessary for the server to
 * manage the connection. This structure is only used internally by the server
 * and can not be accessed from outside the class.
 */
typedef struct
{
  /**
   * \brief client identifier 
   *
   * This ideintifier is automatically assigned by the server when the connection is
   * stablished. This identifier is unique for each connection and it is used to 
   * identify the client to which send or receive information. 
   */ 
  std::string client_id;
  /**
   * \brief identifier of the disconnect event.
   *
   * This events notifies the server side application that the connection to the
   * client has been closed. This event is not the same as the connection closed
   * event provided by the CSocket class. This event is used internally by the
   * server, which activates this one when the disconnection is detected.
   */ 
  std::string disconnect_event_id;
  /**
   * \brief communication soket
   *
   * This field points to the CSocket object used to communicate with the client.
   * This object can only be accessed internally by the class, and the server
   * side application must use the unique identifier assigned to it in order to
   * access it throw the public interface of the server.
   *
   * This object is created and initialized automatically when the connection is
   * stablished, and it is only closed and freed when the free_client() function
   * is called.
   */ 
  CSocket *socket;
}TClient_info_int;

/**
 * \brief Server side socket
 *
 * This class implemnt the server side of a TCP/IP socket communication. It 
 * inherits from the CSocket class for the basic socket communication issues and
 * adds the necessary resources to provide connections to several clients 
 * simultaneously.
 *
 * After creation, it is necessary to call the open() function with the IP address
 * and the desired listining port of the server socket. Then, the config() function
 * is provided with the number of connections that can be buffered waiting for
 * a connection. 
 *
 * At this point the server is properly configured, but it will not accept new 
 * connections until it is started by calling the start() function. After these 
 * steps, the server socket is ready to accept connections and send and receive data 
 * to and from them. The server can be stoped at any time calling the stop() function
 * and restarted. When the server is stopped, all connections are lost.
 *
 * This class provides an event (which can be retrieved with the get_new_connection_event()
 * function) which is activated each time a new client is connected to the server.
 * At this point, a server side application can get all the new client information
 * using the get_new_client() function.
 *
 * This information includes the unique identifier of the socket used to send
 * and receive information because the CSocket object itself is not accessible
 * from outisde the class. It also includes the identifier of an event which is
 * activated when the connection is closed by the client side (disconnect_event_id).
 *
 * When a connection is closed, most of the associated resources in the server
 * side are freed, but it is imperative to call the free_client() function in
 * order to properly free all the resources and avoid memory leaks. This function
 * should only be called after the connection is closed. calling it before, will
 * terminate the connection from the server side.
 *
 * When the maximum number of connections are reached, the server does not accept
 * any more until one of the existing connections is closed.
 *
 */
class CSocketServer : public CSocket
{
  private:
    /**
     * \brief current state of the server
     *
     * This attribute keeps the current state of the server. By default it is 
     * initialized to server_created, and its value automarically changes when
     * the server is opend, configured and started. See the server_state 
     * ennumeration documentation for more information about the possible states
     * of the server and the state transitions.
     */
    server_state state;
    /**
     * \brief maximum number of simultaneous clients
     *
     * This integer has the maximum number of connections allowed for the server.
     * By default its value is set to 1, but it can be changed at any time using
     * the set_max_clients() function. The maximum number of clients can be 
     * retrieved at nay time with the get_max_clients() function.
     */
    int max_clients;
    /**
     * \brief number of clients currently connected
     *
     * This integer has the current number of clients connected to the server. By
     * default it is initialized to 0, and its value is automatically updated when
     * new connections are created or existing connections are closed. The current
     * number of connections can be retrieved with the set_num_current_connections()
     * function.
     */
    int current_clients;
    /**
     * \brief connection counter
     *
     * This static integer is used to assign a unique identifier to each new 
     * connection. This attribute is shared by all servers and it is incremented
     * each time a new connection is set up in any of them, but it is never 
     * decremented. This value is only used internally, and can not be modified 
     * or accessed from outisde the class.
     */
    static int counter;
    /**
     * \brief list of clients connected
     *
     * This list holds the internal information required by the server to manage 
     * each of the connections. By default the list is empty, and it is automatically
     * updated each time a new connections is created or an existing connection
     * is closed. This list is only used internally and can not be modified or 
     * accessed from outside class.
     *
     * See the documentation on the TClient_info_int data type for more
     * information about the elements fo this list.
     */
    std::list<TClient_info_int *> client_list;
    /**
     * \brief mutex object to access the server
     *
     * This object is used to avoid simultaneous access to the shared resources
     * of the server (list of clients) from multiple threads (connection and 
     * disconnection threads and the main thread). This object is initialized at 
     * construction time and used internally in most of the functions.
     */
    CMutex server_access;
    /**
     * \brief identifier of the connection thread
     *
     * This string holds the unique identifier of the connection thread. This 
     * identifier is created at contruction time attaching "_connect_thread" to 
     * the identifier assigned to the server. This identifier can not be modified 
     * or accessed outside the class.
     */
    std::string connect_thread_id;
    /**
     * \brief identifier of the disconnection thread
     *
     * This string holds the unique identifier of the disconnection thread. This 
     * identifier is created at contruction time attaching "_disconnect_thread" to 
     * the identifier assigned to the server. This identifier can not be modified 
     * or accessed outside the class.
     */
    std::string disconnect_thread_id;
    /**
     * \brief identifier of the new connection event
     *
     * This string holds the identifier of the new connection event. This identifier
     * is created at construction time attaching "new_connection_event" to the
     * identifier assigned to the server. This identifier can not be modified outside 
     * the class, but it can be retrieved with the get_new_connection_event_id()
     * function to wait for new connections outside the class.
     */
    std::string new_connection_event_id;
    /**
     * \brief identifier of the update list event
     *
     * This string holds the identifier of the update list event. This identifier
     * is created at construction time attaching "update_list_event" to the
     * identifier assigned to the server. This event is only used inside the class
     * and can not be accessed or modified outside it.
     */
    std::string update_list_event_id;
    /**
     * \brief identifier of the event to terminate the connect thread
     *
     * This event is used to signal the thread that is waiting for new connections
     * to terminate its operation. This thread periodically checks the state of this
     * event.
     *
     * This event is created when the server is created, but it is only activated 
     * when the stop_server() function is called or the object is destroyed. This
     * event is for internal use only, and can not be accessed outside the class.
     */
    std::string finish_connect_thread_event_id;
    /**
     * \brief identifier of the event to terminate the disconnect thread
     *
     * This event is used to signal the thread that is waiting for disconnections
     * to terminate its operation. This thread periodically checks the state of this
     * event.
     *
     * This event is created when the server is created, but it is only activated 
     * when the stop_server() function is called or the object is destroyed. This
     * event is for internal use only, and can not be accessed outside the class.
     */
    std::string finish_disconnect_thread_event_id;
    /** 
     * \brief queue of clients waiting to be retrieved
     *
     * This queue temporary holds the information of the new connections stablished
     * to the server. This queue is empty by default and it is filled each time a new
     * connection is created. The queue is emptied by calling the get_new_client()
     * function when the new_connection_event_id event is activated, which returns 
     * the necessary information to handle the connection to the server side 
     * application. This queue can not be modified or accessed outside the class.
     *
     * This event can be activated several times if multiple connections are created
     * without being acknowledge by the server side application.
     */
    std::queue<TClient_info *> new_clients;
  protected:
    /**
     * \brief server connection IP address and port
     *
     * This strcture has the IP address and listening port of the server. By default
     * the IP address uis empty and the port is set to -1. When the open() function is
     * called successfully, this values are updated to the values provided to the
     * function. 
     *
     * These values can not be directly modified nor accessed outside the class.
     */
    TSocket_info info;
    /**
     * \brief connection thread function
     *
     * This is the main function for the connection thread. This thread is waiting 
     * for new connections continuously. When a new connection is created, it first
     * created a new CSocket object associated to the new socket file descriptor
     * to handle all the communication issues and generates both the internal 
     * (TClient_info_int) and external (TClient_info) structures with all the 
     * necessary information.
     *
     * The CSocket object can only be accessed from inside the class, and the
     * server side application must use the client identifier provided by the
     * get_new_client() function to access it. 
     *
     * After that, the new_connection event is activated to signal the server side
     * application that a new connection is ready. If several connections are 
     * created without being acknowledge by the server side application, this event
     * remains active until all the new client information is retrieved.
     *
     * This thread does not throw any exception.
     *
     * \param param a pointer to the CSocketServer object to which the thread is
     *              associated. This parameter is used to access the internal
     *              attriobutes and functions of the class since the thread itself
     *              is defined as static.
     */
    static void *connect_thread(void *param);
    /**
     * \brief disconnection thread function
     *
     * This is the main function of the disconnection thread. This thread waits 
     * for the connection closed event of each of the active connections. When
     * one or more of these events are activated, it frees most of the internal 
     * resources associated with the connection and activates the corresponding
     * disconnect event to signal the server side application that the
     * connection is no longer available.
     *
     * It is the server side application the one responsible of freeing the rest
     * of the resources associated with the closed connection by calling the
     * free_client() function when the disconnect event is activated.
     *
     * If there are no active connections, this thread is inactive. As new 
     * connections are created, this thread periodically updates the wait event 
     * list to match the current active connections.
     *
     * This thread does not throw any exception.
     *
     * \param param a pointer to the CSocketServer object to which the thread is
     *              associated. This parameter is used to access the internal
     *              attriobutes and functions of the class since the thread itself
     *              is defined as static.
     */
    static void *disconnect_thread(void *param);
    /**
     * \brief function to bind the desired IP address and port to the socket
     *
     * This function is automatically called when the base class open() function
     * is called. This function binds the server socket to the desired IP
     * address and port provided to the function. The IP address must coincide
     * with the IP address assigned to the computer where the server is being
     * executed, and the port can not be used by nay other application.
     *
     * If successful, the internal state of the server is changed to server_binded.
     * After calling this function, it is necesary to call the config() function
     * to properly configure the server.
     *
     * \param comm_dev a pointer to a valid TSocket_info strcture with the server
     *                 IP address and desired listening port. See the documentation
     *                 on the TSocket_info structure for more information on the
     *                 IP address and port format.
     */
    virtual void hard_open(void *comm_dev=NULL);
    /**
     * \brief function to configure the length of the connection buffer
     *
     * This function is automatically called when the base class config() function
     * is called. This function sets the maximum number of waiting connections 
     * that are allowed. 
     *
     * If successfull, the internal state of the server is changed to server_listening.
     * After calling this function, the server is properly configured, but it is still
     * not ready to accept new connections or send and receive data. To do that, it
     * is necessary to call the start() function.
     *
     * \param config a pointer to an integer value with the number of waiting
     *               connections allowed. This value must be a positive integer.
     */
    virtual void hard_config(void *config=NULL);
    /**
     * \brief function to wait for communication events
     *
     * This function is only provided for compatibility since the server socket never 
     * receives data from a client (a new socket is created for that purpose). However, 
     * changes in the state of the socket may generate unexpected errors that the 
     * CSocket hard_wait_comm_event() function can not handle.
     *
     * Therefore, this function overrides the base class function and block the thread
     * indefinetely. When the server is destroyed, the thread is killed and this 
     * function is aborted.
     *
     * \return this function will block and never return any value. 
     */
    virtual int hard_wait_comm_event(void);
    /**
     * \brief function to close the server
     *
     * This function will first stop the server and also close any connection with a 
     * client which is still active, freeing all the associated resources. If 
     * successfull this function changes the state of the server to server_created.
     *
     * If this function is called, in order to set up the server again, it will be
     * necessary to call the open() and config() functions to reconfigure it and also
     * the start() function to accept and handle new connections.
     */
    virtual void hard_close(void);
  public:
    /**
     * \brief constructor
     *
     * This constructor created the connect and disconnect threads and attach them
     * to the corresponding functions, but does not start any of them. It also
     * creates the new connection event. The socket identifier is used to initialize
     * the identifier for both threads and events, and each server must have a 
     * different one.
     *
     * In this function all the internal attributes are initialized by default.
     *
     * \param sock_id a null terminated string with the identifier of the server.
     *                This string must not have any white spaces and each server
     *                must have a different one, since it is used to generate the
     *                unique identifier for both threads and events.
     */
    CSocketServer(const std::string &sock_id);
    /**
     * \brief function to set the maximum number of simultaneous clients
     *
     * This function sets the maximum number of clients connected any given time.
     * This function can be called any time, but if the new maximum value is 
     * smaller than the current number of connections, this function will throw
     * a CSocket exception instead of closing the exceeding connections.
     *
     * If the new value is bigger than the previous one, there is no problem.
     * However, it is better to set this value before calling the start()
     * function and, if it is necessary to change the maximum number of clients
     * connected, first stop the server and then change it.
     *
     * \param max_clients a positive integer with the maximum number of clients
     *                    connected any given time. 
     */
    void set_max_clients(int max_clients);
    /**
     * \brief function to get the maximum number of simultaneous clients
     *   
     * This function return the maximum number of clients that can be connected 
     * to the server at any given time. 
     *
     * \return a positive integer with the maximum number of clients connected
     *         to the server any given time.
     */
    int get_max_clients(void);
    /**
     * \brief function to get the number of clients currently connected
     *
     * This returns the number of clients currently connected to the server. This
     * number is always between 0 and the maximum number of clients allowed. 
     *
     * \return a non-negative integer with the number of clients currently
     *         connected to the server.
     */
    int get_num_current_clients(void);
    /**
     * \brief function to start the server
     *
     * This function starts the internal threads which allow the server to detect 
     * new connections. Before calling this function, the server will not be able 
     * to accept any new connections. If successfull this function changes the state
     * of the server to server_on.
     *
     * After calling this function, the server side application should wait for the
     * activation of the new connection event. Once activated, the new client
     * information can be retrieved by calling the get_new_client() function. 
     *
     * At this point, the receive event and the disconnet event should be monitored to
     * detect the reception of new data or the disconnection of the client, whatever
     * happens first. If new data is received, the get_num_data_from() and read_from()
     * functions can be used to get it. If the connection has been closed, the 
     * free_client() function should be called to free all the associated resources.
     *
     * If successfull, this function changes the state of the server to server_on.
     * This function throws a CSocketException in case of any error.
     */
    void start_server(void);
    /**
     * \brief function to stop the server
     *
     * This function stops the normal operation of the server. It closes any active
     * connection that may still exists and frees all the associated resources. In 
     * this case, however, the server does not loose its configuration, so in order
     * to start accepting connections again, it is only necessary to call the 
     * start() function.
     *
     * If successfull, this function changes the state of the server to 
     * server_listening. This function throws a CSocketException in case of any 
     * error;
     */
    void stop_server(void);
    /**
     * \brief function to send some data to all clients
     *
     * This function is used to send any data to all the active connections on the
     * server. In this case no client identifier is provided because the data is
     * sequentially sent to all available clients.
     *
     * This function throws a CSocketException in case of any error.
     *
     * \param data a pointer to an unsigned char vector with the information to
     *             send. The memory for this parameter must be allocated before
     *             calling this function, and its length must coincide with the
     *             value of the second parameter.
     *
     * \param len a positive integer with the length of the data to be sent. This
     *            value must coincide with the actual length of the vector passed
     *            as first argument.
     */
    void broadcast(unsigned char *data, const int len);
    /**
     * \brief function to send data to a single client
     *
     * This function is used to send data to a single client. It uses the client
     * identifier returned by the get_new_client() function to identify the
     * desired client connection. If the specified client does not exist, this
     * function throws a CSocketException.
     *
     * \param sock_id the identifier of the desired client. This value must be
     *                one of the identifier returned by the get_new_client()
     *                function which is still active.
     *
     * \param data a pointer to an unsigned char vector with the information to
     *             send. The memory for this parameter must be allocated before
     *             calling this function, and its length must coincide with the
     *             value of the third parameter.
     *
     * \param len a positive integer with the length of the data to be sent. This
     *            value must coincide with the actual length of the vector passed
     *            as second argument.
     *
     * \return the number of bytes actually written to the client. In a standard 
     *         case, this value should be equal to the value of the third parameter,
     *         otherwise, and error has ocurred.
     */
    int write_to(const std::string &sock_id, unsigned char *data, const int len);
    /**
     * \brief function to get data from a single client
     *
     * This function is used to get data from a single client. It uses the client
     * identifier returned by the get_new_client() function to identify the 
     * desired client connection. If the specified client does not exist, this
     * function throws a CSocketException.
     *
     * \param sock_id the identifier of the desired client. This value must be
     *                one of the identifier returned by the get_new_client()
     *                function which is still active.
     *
     * \param data a pointer to an unsigned char vector where the information 
     *             received is stored. The memory for this parameter must be 
     *             allocated before calling this function, and its length must 
     *             coincide with the value of the third parameter.
     *
     * \param len a positive integer with the length of the data to be read. This
     *            value must coincide with the actual length of the vector passed
     *            as second argument. The get_num_data_from() function can be 
     *            used to know how many data is available at any time.
     *
     * \return the number of bytes actually read from the client. In a standard 
     *         case, this value should be equal to the value of the third parameter,
     *         otherwise, and error has ocurred.
     */
    int read_from(const std::string &sock_id, unsigned char *data, const int len);
    /**
     * \brief function to get the number of available data from a single client
     *
     * This function is used to get the ammount of data available from a single 
     * client. It uses the client identifier returned by the get_new_client() 
     * function to identify the desired client connection. If the specified 
     * client does not exist, this function throws a CSocketException.
     *
     * \param sock_id the identifier of the desired client. This value must be
     *                one of the identifier returned by the get_new_client()
     *                function which is still active.
     *
     * \return the number of bytes currently available at the desired client. This
     *         value can be used as the third parameter to the read_from()
     *         function.
     */
    int get_num_data_from(const std::string &sock_id);
    /**
     * \brief function to get the identifier of the new connection event
     *
     * This function return the identifier of the new connection event. This
     * event can be used to wait for new connections once the server has been
     * started with the start() function. There exist a single event for each 
     * server that is activated as many times as new clients are pending to be
     * retrieved.
     *
     * After this event is active, the server side application should call the
     * get_new_client() function in order ro get all the necessary information
     * to handle the new connection. 
     *
     * \return a string with a copy of the new connection event identifier. 
     */
    std::string get_new_connection_event_id(void);
    /**
     * \brief function to get the information about the new connection
     *
     * This function is used to get all the important information about a new
     * connection. See the documentation on the TClient_info structure for more
     * information about the parameters returned by this function. 
     *
     * This function should be called as long as the new connection event is 
     * active, which means that there is a new client connected and waiting.
     *
     * \return a pointer to an structure with information about the new 
     *         connection. The memory necessary for this structure is allocated
     *         in this function, and it is necessary to call the free_client()
     *         function to properly free it and all its associated resources.
     */
    TClient_info *get_new_client(void);
    /**
     * \brief function to free the resources associated with a client
     *
     * This function is used to properly free all the resources associated with a
     * connection once is has been closed. This function should only be called 
     * after the disconnect event for the corresponding connection has been
     * activated. Otherwise, the connection will be closed and the client will be
     * disconnected.
     *
     * This function actually closes the socket accociated to the connection and 
     * destroys it, and also destroys all the events associated with it. Finally, 
     * it also, frees the information structure passed as an argument so the
     * server side of the application does not have to do that.
     *
     * \param info a pointer to a valid TClient_info structure which corresponds 
     *             to an active connection. This structure should be the one
     *             returned by the get_new_client() function.
     */
    void free_client(TClient_info *info);
    /**
     * \brief destructor
     *
     * This destructor first stops the server, if it is running, and closes all 
     * the active connections with clients. This destructor does not free all the
     * resources associated with each connection, so its better to first close all
     * the connections using the free_client() function and then destroy the 
     * server object.
     */
    virtual ~CSocketServer();
};

#endif
