#ifndef _COMM_H
#define _COMM_H

#include <list>
#include "eventserver.h"
#include "threadserver.h"
#include "mutex.h"

typedef enum {created,configured,opened,sending} comm_states;

/**
 * \brief Implementation of a generic communication devidei
 * 
 * This class implements the basic features of a communication device. All
 * standard communication devices (pipes, serial ports, sockets, etc...) will
 * inherit from this base class and only need to define the specific functions
 * to open (hard_open()), configure (hard_config()), read (hard_read(), write
 * (hard_write()), get the number of available data (hard_get_num_data()), 
 * close (hard_close()) and wait for communication events (hard_wait_comm_event())
 * on the device.
 *
 * The interface provided by this class includes two events (data reception,
 * and error) and one data queue where the received data is temporarly stored. 
 * The events can be accessed through the event handler (CEventServer) by any 
 * other object, and two functions are provided to retrieve their 
 * identifiers (get_rx_event_id(), and error_event_id()).
 *
 * The recive event indicates if the device has received data and it remains
 * active while there is data in the internal queue to be read. When the receive 
 * queue is empty, the event is reset. Finally the error event indicates that an
 * error ocurred. Use the get_error() function to retrieve the error code.
 *
 * When created the communication device starts a thread that is actually the 
 * one responsible of handling the communication events (received data, 
 * and errors). This thread is used exlcusively inside the class and can not be
 * accessed outside it. This thread uses the wait function provided by the 
 * inherited class to wait for any communication events.
 *
 * Most devices in Linux are referenced by a file descriptor for both read and
 * write operations, or also, two different file descriptors, one for read 
 * operations, and the other one for write operations. However, other devices
 * may provide different methods of interfacing them, so no specific device
 * handler is provided by this class. It is the inherited class that must provide
 * it, together with the functions to open it, configure it, read from and write 
 * to it and close it.
 *
 * To open the device, the user must call the base class open() function and also
 * provide a function (hard_open()) to actually open the communication device. 
 * The user provided function is automatically called by the base class open()
 * function and it is responsible of initializing the device handler.
 *
 * After that, the device must be configured. Similarly to what happen with the
 * open() function, the inherited class must provide the hard_config() function
 * to perform all the required configuration steps. Both the open() and config()
 * function have a void * parameter to allow the inherited class to pass through 
 * any data structure. This parameter is passed to the corresponding inherited 
 * class function (hard_open() or hard_config() respectively).
 *
 * Only after configuration, it is possible to read from and write to the device.
 * Any previous attempt to do so will result in an error. The inherited class
 * must also provide functions to actually read from (hard_read()) and write to
 * (hard_write()) the device. To close the communication device, it is necessary
 * to call the close() function which ends the thread and flushes any data still
 * inside the transmission and reception queues. Also this function calls the 
 * hard_close() function to actually close the device handle.
 *
 * The events and the thread itself are not destroyed until the communciation 
 * device object is destroyed. It is possible to close the device at any time.
 *
 * For a propper operation, any inherited class must provide an implementation
 * for the hard_open(), hard_config(), hard_read(), hard_write(), hard_close(),
 * hard_get_num_data() and hard_wait_comm_event() functions. This functions are
 * automatically called by the base class and must not be called directly.
 *
 * Any communication device is driven by an internal state machine, whose state is
 * automatically changed when the different functions are called. The next figure
 * shows the different states and also the state transition conditions.
 *
 *  \image html comm_states.png
 *  \image latex comm_states.eps "Communication device states" width=10cm
 *
 */
class CComm
{
  protected:
    /**
     * \brief receive event identifier
     *
     * This string has a unique identifier of the reception event that is used 
     * through out the code to take action on the desired event. This string is 
     * initialized at construction time using the identifier of the communication 
     * device provided to the constructor and can not be modified afterwards. The
     * function get_rx_event_id() can be used to retrieve this identifier.
     */
    std::string rx_event_id;
    /**
     * \brief error event identifier
     *
     * This string has a unique identifier of the error event that is used 
     * through out the code to take action on the desired event. This string is 
     * initialized at construction time using the identifier of the communication 
     * device provided to the constructor and can not be modified afterwards. The
     * function get_error_event_id() can be used to retrieve this identifier.
     */
    std::string error_event_id;
    /**
     * \brief communication thread identifier
     *
     * This string has a unique identifier of the communication thread that is used 
     * through out the code to take action on the desired thread. This string is 
     * initialized at construction time using the identifier of the communication 
     * device provided to the constructor and can not be modified afterwards. This
     * thread is only accessible from inside the class so it is not possible to
     * retrieve its identifier.
     */
    std::string comm_thread_id;
    /** 
     * \brief communication error message
     *
     * This string has the information message of any error that could happen on
     * the communication device. By default it is initialized to NULL, and it is 
     * only created when there is an error. The error message is automatically 
     * created when an error is detected by the communication thread, and it can
     * be retrieved by calling the get_error() function.
     */ 
    std::string error_msg;
    /** 
     * \brief received data queue
     *
     * This dynamic queue stores all the data that is recieved by the communication
     * device until it is read by the user. There is no limit in the number of bytes
     * stored in this list except for the physical memory available. The data 
     * received through the serial port is automatically put into the queue by the 
     * communication thread, and the function read() is used to retrieve it. The 
     * function get_num_data() returns the number of bytes in this queue. By default,
     * this queue is empty.
     */
    std::list<unsigned char> receive_queue;
    /**
     * \brief current state of the communication device
     * 
     * This variable keeps the current state of the communication device. The state
     * of the device is changed automatically by the class functions and can not be 
     * modified outside it. The possible states of the communication device are:
     *
     *  - created: when the object has been just created.
     *  - opened: after opening the device with the open() function.
     *  - configured: after calling the config() function.
     *  - sending: when the device is busy sending data.
     *
     * It is possible to retrieve the current state of the communication device at
     * any time using the get_state() function.
     */
    comm_states state;
    /**
     * \brief Reference to the unique event handler
     *
     * This reference to the unique event handler is initialized when an object of
     * this class is first created. It is used to create and handle all the 
     * communication events. The object pointed by this reference is shared by all
     * objects in any application.
     */  
    CEventServer *event_server;
    /**
     * \brief Reference to the unique thread handler
     *
     * This reference to the unique thread handler is initialized when an object of
     * this class is first created. It is used to create and handle all the 
     * communication threads. The object pointed by this reference is shared by all
     * objects in any application.
     */ 
    CThreadServer *thread_server;
    /**
     * \brief A unique identifier for the obejct
     *
     * This string has a unique identifier of the object that is used throug
     * out the code to take action on the desired object. This string is
     * initialized at contruction time and can not be modified afterwards.
     */
     std::string comm_id;
    /**
     * \brief Communication mutual exclusion object
     *
     * This object is intended to be used by to handle the access to the shared 
     * communication resource defined by the inherited classes. Using this mutex
     * multiple simultaneous read or write operations on the communication device 
     * are avoided which would result in data corruption or unexpected errors.
     * This object is initialized at contruction time.
     */
    CMutex access_comm;
    /** 
     * \brief Thread function
     * 
     * This is the main function executed by the communication device to handle 
     * the communication events. This function waits for either a receive or error
     * events permanently, and the calls the appropriate function to handle the 
     * events (on_receive() and on_error() respectively).
     *
     * Except for the time where this function is actually waiting for an event 
     * to get active, it locks the user mutex of the base class in order to avoid
     * that other threads interfere with the correct execution of the function. 
     * The mutex is freed when the function is waiting to allow other threads to
     * access the class shared resources.
     *
     * When data is received, this function automatically reads it from the physical
     * communication device and stores it into the internal receive queue. Then the 
     * receive event is activated to awake any waiting thread.
     *
     * In case of an error, the error event is activated. To retrieve the error 
     * message, it is necessary to use the get_error() function. In case of an error,
     * the thread is terminated, but no exception is thrown because there is no way 
     * to catch it.
     *
     * \param param a reference to the communication device object that is associated
     * to the thread. This reference is necessary to access the internal attributes
     * of the class.
     */
    static void *comm_thread(void *param);
    /**
     * \brief function to handle the reception events
     *
     * This function handles the receive events. When called, it first gets how many 
     * bytes have been received. Then all the bytes are read from the physical 
     * communication device and stored into the internal receive queue. If the event
     * is not already set, it is activated to indicate to any waiting thread that
     * data is available.
     *
     * This function is executed only by the communication thread when new data is 
     * received, so its execution can not be interrupted by other threads trying to
     * access the class shared resources because the mutex is already locked by the
     * communications thread function
     *
     * This function throws a CCommException in case of any error.
     */
    void on_receive(void);
    /** 
     * \brief function to handle the errors
     *
     * This function handles any error in the communication device. This function
     * creates an error message with the information of the error, and the activates
     * the error event and throws an exception in order to finish the thread. It is 
     * important to note than the thread is terminated if there is any error.
     *
     * This function is executed only by the communication thread when new data is 
     * received, so its execution can not be interrupted by other threads trying to
     * access the class shared resources because the mutex is already locked by the
     * communications thread function
     *
     * This function throws a CCommException in case of any error.
     */
    void on_error(void);
    /**
     * \brief Function to set the object identifier
     *
     * This function sets the unique identifier of each object. This function
     * is protected, and therefore can only be executed inside the class, because
     * the identifier can not be modified after construction of the object.
     *
     * This function throws a CCommException if there is any error.
     *
     * \param comm_id A null terminated string which identified the 
     *                communication device. This string is used to create a
     *                unique identifier for all the threads and events of the
     *                class.
     */
     void set_id(const std::string& comm_id);
     /**
      * \brief Function to actually open the device
      *
      * This function is called automatically when the base class open() function
      * is called. It must create a handle to the communication device in order to 
      * be used in the future. The device handle must be provided by any inherited 
      * class since it is device dependant.
      *
      * This class accepts a generic parameter (void *) to allow the user to pass 
      * to the function any data structure. This parameter comes from the base
      * class open() function, but no action is performed on it.
      *
      * This function can throw any CCommException object exception or else any
      * exception class defined by the inherited class.
      *
      * \param comm_dev a generic pointer (void *) to the data structure needed to
      *                 identify the communication device to open and any other
      *                 required information. This parameter may be NULL if not
      *                 needed.
      */ 
     virtual void hard_open(void *comm_dev=NULL)=0;
     /**
      * \brief Function to actually configure the device
      * 
      * This function is called automatically when the base class config() function
      * is called. It must configure the communication device as required by the 
      * application.
      *
      * This class accepts a generic parameter (void *) to allow the user to pass 
      * to the function any data structure. This parameter comes from the base
      * class open() function, but no action is performed on it.
      *
      * This function can throw any CCommException object exception or else any
      * exception class defined by the inherited class.
      *
      * \param config a generic pointer (void *) to the data structure needed to
      *               configure the communicationd device. This parameter may be
      *               NULL if not needed.
      */ 
     virtual void hard_config(void *config=NULL)=0;
     /**
      * \brief Function to actually read from the device
      *
      * This function is automatically called when the new data received is activated.
      * The read() function from the base class gets data from the internal queue, so
      * this function is not used. It must try to read the ammount of data specified 
      * and store it in the data buffer provided without blocking. Also, it must 
      * return the number of bytes actually read from the devicve, since they may be
      * different than the desired value.
      *
      * This function can throw any CCommException object exception or else any
      * exception class defined by the inherited class.
      *
      *  \param data a reference to the buffer where the received data must be 
      *  copied. The necessary memory for this buffer must be allocated before 
      *  calling this function and have enough size to store all the desired data.
      *  If this buffer is not initialized, the function throws an exception.
      *
      *  \param len a positive interger that indicates the number of byte to be
      *  read from the communication device. This value must be at most the length 
      *  of the data buffer provided to the function.
      *
      *  \return an integer with the number of bytes actually read. These number 
      *  coincide with the desired number if there is enough data in the internal 
      *  queue, but it could be smaller if not.
      */ 
     virtual int hard_read(unsigned char *data, int len)=0;
     /**
      * \brief Function to actually write to the device
      *
      * This function is automatically called when the base class write() function
      * is called. It must try to write the desired ammount of data to the communication 
      * device without blocking. Also it must return the number of bytes actually
      * written to the communication device since they may be different from the
      * desired value.
      *
      * This function can throw any CCommException object exception or else any
      * exception class defined by the inherited class.
      *
      *  \param data a reference to the buffer with the data must be send to the 
      *  device. The necessary memory for this buffer must be allocated before 
      *  calling this function and have enough size to store all the desired data.
      *  If this buffer is not initialized, the function throws an exception.
      *
      *  \param len a positive interger that indicates the number of byte to be
      *  written to the communication device. This value must be at most the length 
      *  of the data buffer provided to the function.
      *
      *  \return an integer with the number of bytes actually written. These number 
      *  coincide with the desired number if there is enough data in the internal 
      *  queue, but it could be smaller if not.
      */ 
     virtual int hard_write(unsigned char *data, int len)=0;
     /**
      * \brief Function to actually get the number of bytes availables
      *
      * This function is called when the new data received event is activated.
      * It must get the number of data bytes available from the communication
      * device and return its value without blocking.
      *
      * This function can throw any CCommException object exception or else any
      * exception class defined by the inherited class.
      *
      * \return an integer with the number of bytes available in the receive queue. 
       * This value can be 0 if there is no data available, but there is ni upper
      * limit in its value.
      */ 
     virtual int hard_get_num_data(void)=0;
     /**
      * \brief Function to actually wait for a given communication event
      *
      * This function is called in the internal communciation thread to wait for 
      * any event on the communuication device. It must check for any event on the
      * device (reception or error) and return the corresponding identifier. When 
      * an event is activated, this function must return to allow the base class 
      * to handle it, and the it is called again to wait for the next event.
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
     virtual int hard_wait_comm_event(void)=0;
     /**
      * \brief Function to actually close the device
      *
      * This function is called when the base class close() funciton is called. It
      * must free the device handle initialized by the open() function and also free
      * any other resource allocated by the inherited class.
      *
      * This function can throw any CCommException object exception or else any
      * exception class defined by the inherited class.
      */ 
     virtual void hard_close(void)=0;
  public:
    /** 
     * \brief Constructor
     * 
     * This constructor creates the receive, send and error events and also the 
     * thread to handle these events. By default all the events are reset. After
     * creating the object is already possible to get the event identifiers for 
     * the receive and error events.
     *
     * The thread is attached to the function to be executed, but it is not 
     * started yet. It is only started when the open() function is called.  
     *
     * Also the error message is not initialized (set to NULL) at construction 
     * time. This message is only initialized when there is an error on the
     * communication device.
     *
     * By default, the intial state of the communication device is created.
     *
     * \param comm_id A null terminated string which identified the 
     *                communication device. This string is used to create a
     *                unique identifier for all the threads and events of the
     *                class.
     */
    CComm(const std::string& comm_id);
    /** 
     * \brief Function to retrieve the object identifier
     *
     * This function returns the object identifier of the object as a null
     * terminated string.
     *
     * This function thows a CCommException if there is any error.
     *
     * \return A refernce to a non allocated memory space where the
     * identifier of the object is to be copied. The necessary memory is allocated
     * internally to match the size of the identifier. The calling process is
     * responsible of freeing the allocated memory.
     */
    std::string& get_id(void);
    /**
     * \brief function to open the communication device
     *
     * This function starts the communication thread that has been created at
     * construction time if the device is in the created state. Otherwise, an
     * exception is thrown because the device should be alredy opened. If it is
     * necessary to open a new communication device using a previously 
     * initialized CCom  object, it is necessary to first close the communication 
     * device with the close() function and the call this function again. If 
     * successfull, this function changes the current state of the device to
     * opened.
     *
     * This function does not actually open any communication device, since this
     * class is a generic placeholder for communication devices. It is the 
     * inherited class which has to provide a hard_open() function that actually 
     * opens the desired communication device. The inherited class hard_open() 
     * function must initialize the device handler.
     * 
     * This function throws a CCommException if there is any error.
     */
    void open(void *comm_dev=NULL);
    /**
     * \brief function to configure the communication device
     *
     * This function only changes the current state of the communication device
     * if it is not already configured, but does not configure any of the 
     * device parameters since they greatly vary from one communication device to
     * another. If the device has not been previously opened, this function 
     * throws an exception.
     *
     * It is the inherited class that has to provide a hard_config() function that 
     * actually configures the particular parameters of the communication device. 
     *
     * This function throws a CCommException if there is any error.
     */
    void config(void *config=NULL);
    /**
     * \brief function to read from the communication device
     *
     * This function tries to read a given number of bytes from the communication
     *  device if the device has been opened and configured. Otherwise it throws 
     *  an exception. If the number of bytes currently in the receive queue is less
     *  than the desired number, the function returns all the available data and
     *  also the number of bytes actually read. 
     *
     *  If the number of bytes in the queue is greater or equal than the desired 
     *  number, the function returns all the required data. In both cases, the 
     *  function will never block. When read, the bytes are removed from the
     *  internal queue, so they can not be read again. Use the get_num_data()
     *  function to get how many bytes are available in the receive queue.
     *
     *  If the internal receive queue gets empty after a read operation, the
     *  receive event is cleared to indicate that there is no more data available.
     *  This event will be set again by the communication thread when new data
     *  is received.
     *
     *  This function throws a CCommException if there is any error.
     *
     *  \param data a reference to the buffer where the received data must be 
     *  copied. The necessary memory for this buffer must be allocated before 
     *  calling this function and have enough size to store all the desired data.
     *  If this buffer is not initialized, the function throws an exception.
     *
     *  \param len a positive interger that indicates the number of byte to be
     *  read from the communication device. This value must be at most the length 
     *  of the data buffer provided to the function.
     *
     *  \return an integer with the number of bytes actually read. These number 
     *  coincide with the desired number if there is enough data in the internal 
     *  queue, but it could be smaller if not.
     */
    int read(unsigned char *data,int len);
    /**
     * \brief function to write to the communication device
     *
     * This function tries to write a given number of bytes to the communication
     * device if the device has been opened and configured. Otherwise it throws 
     * an exception. If the device is currently sending data, the new data provided
     * to this function is temporary stored into the internal send queue to be
     * trasmited when the device is ready. Otherwise, the information is send
     * immediately.
     *
     * If the new data is send immediatelly, the send event is reset to indicate 
     * that the device is not ready to send data, and any further write operation
     * will temporary store the data into the internal queue. This event is set
     * again by the communication device when a transmission is ended and there
     * is no data in the send queue.
     *
     * This function throws a CCommException if there is any error.
     *
     *  \param data a reference to the buffer with the data must be send to the 
     *  device. The necessary memory for this buffer must be allocated before 
     *  calling this function and have enough size to store all the desired data.
     *  If this buffer is not initialized, the function throws an exception.
     *
     *  \param len a positive interger that indicates the number of byte to be
     *  written to the communication device. This value must be at most the length 
     *  of the data buffer provided to the function.
     *
     *  \return an integer with the number of bytes actually written. These number 
     *  coincide with the desired number if there is enough data in the internal 
     *  queue, but it could be smaller if not.
     */
    int write(unsigned char *data, int len);
    /**
     * \brief function to get the number of bytes in the reception queue
     *
     * This function returns the number of byte currently available in the internal
     * receive queue.
     *
     * \return an integer with the number of bytes available in the receive queue. 
     * This value can be 0 if there is no data available, but there is ni upper
     * limit in its value.
     */
    unsigned int get_num_data(void);
    /**
     * \brief function to get the receive event identifier
     * 
     * This fucntion returns the receive event identifier as a null terminated 
     * string that can be used to access the event from any thread.
     *
     * This function throws a CCommException in case of any error.
     *
     * \return A reference to a non-allocated memory space where the 
     * identifier of the event is to be copied. The necessary memory is allocated
     * internally to match the size of the identifier. The calling process is 
     * responsible of freeing the allocated memory.
     *
     */
    std::string& get_rx_event_id(void);
    /** 
     * \brief function to get the error event identifier
     *
     * This fucntion returns the error event identifier as a null terminated 
     * string that can be used to access the event from any thread.
     *
     * This function throws a CCommException in case of any error.
     *
     * \return A reference to a non-allocated memory space where the 
     * identifier of the event is to be copied. The necessary memory is allocated
     * internally to match the size of the identifier. The calling process is 
     * responsible of freeing the allocated memory.
     */
    std::string& get_error_event_id(void);
    /**
     * \brief function to get the error message
     *
     * This function returns the error message of the error that caused the 
     * communication thread to end. This function will return always a NULL 
     * pointer except when an error has ocurred. 
     *
     * This function throws a CCommException in case of any error.
     *
     * \return A reference to a non-allocated memory space where the 
     * error message is to be copied. The necessary memory is allocated
     * internally to match the size of the identifier. The calling process is 
     * responsible of freeing the allocated memory.
     */
    std::string& get_error(void);
    /** 
     * \brief function to get the current state of the comunication device
     * 
     * This function returns the current state of the communication device. The
     * state is automatically updated by the class itself and can not be modified
     * outside the class.  
     *
     * This function throws a CCommException in case of any error.
     *
     * \return the current state of the communication device. The possible state 
     * are:
     *  - created: when the object has been just created.
     *  - opened: after opening the device with the open() function.
     *  - configured: after calling the config() function.
     */
    int get_state(void);
    /**
     * \brief function to close the communication device
     *
     * This function closes the communication device. This operation consists on
     * terminating the communication thread to avoid the event from being set or
     * reset by it. Even if the particular communication device is opened by an
     * inherited class, this function closes it. So the inherited class does not
     * have to do it. Finally, any data in both the receive and the send queues 
     * are flushed.
     *
     * This function does not destroy the events or the thread objects because 
     * the device can still be opened again. These objects are only destroyed 
     * when the object itself is destroyed. After calling this function, the
     * current state of the communication device is set to created whatever
     * the old state was.
     *
     * This function throws a CCommException in case of any error.
     */
    void close(void);
    /**
     * \brief destructor
     *
     * This function destroys the communication object. First, it calls the
     * close() function to finish the thread and flush any remaining data. It
     * then destroys all the events and the communication thread.
     *
     * This function throws a CCommException in case of any error.  
     */
    virtual ~CComm();
};

#endif
