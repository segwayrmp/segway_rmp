#ifndef _FTDI_MODULE_H
#define _FTDI_MODULE_H

#include <ostream>
#include "ftd2xx.h"
#include "comm.h"

/**
  \brief structure to hold the configuration parameters of the FTDI device

  The necessary configuration parameters of the supported FTDI devices are:

  - baud rate (only used on serial devices)
  - number of bits per word (only used on serial devices)
  - number of stop bits (only used on serial devices)
  - parity type (only used on serial devices)
  - read and write timeouts (used on all devices)
  - latency timer (used on all devices)

  These parameters must be configured after the device has been opened by
  calling the open() function and before reading or writing data from or to
  the device.
 
*/
typedef struct TFTDIconfig 
{
  /**
   * \brief desired baud rate
   *
   * This parameter specifies the desired baud rate in bits per second of the
   * device. This parameter only has meaning when the FTDI device is of the
   * serial kind. Otherwise it is not used and it can be set to -1 to avoid 
   * its initialization on the driver.
   *
   * The possible values for this parameter are:
   *
   * - 300
   * - 600
   * - 1200
   * - 2400
   * - 4800
   * - 9600
   * - 14400
   * - 19200
   * - 38400
   * - 57600
   * - 115200
   * - 230400
   * - 460800
   * - 921600
   *
   */ 
  int baud_rate; 
  /**
   * \brief desired word length
   *
   * This parameter specifies the desired length of each packet in bits. This
   * parameter only has meaning when the FTDI device is of the serial kind.
   * Otherwise it is not used and it can be set to -1 to avoid its initialization
   * on the driver.
   *
   * The possible values for this parameter are integers from 5 to 8.
   */ 
  int word_length;
  /**
   * \brief desired number of stop bits
   *
   * This parameter specifies the desired number of stop bits of each packet. This
   * parameter only has meaning when the FTDI device is of the serial kind.
   * Otherwise it is not used and it can be set to -1 to avoid its initialization
   * on the driver.
   *
   * The possible values for this parameter are:
   *
   * - 0 for 1 stop bits
   * - 1 for 1.5 stop bits
   * - 2 for 2 stop bits
   */ 
  int stop_bits;
  /**
   * \brief desired parity type
   *
   * This parameter specifies the desired kind of parity to be used. This parameter
   * only has meaning when the FTDI device is of the serial kind. Otherwise it is 
   * not used and it can be set to -1 to avoid its initialization on the driver.
   *
   * The possible values for this parameter are:
   *
   * - 0 for no parity
   * - 1 for odd parity
   * - 2 for even parity
   * - 3 for mark parity
   * - 4 for space parity 
   */  
  int parity;
  /**
   * \brief desired timeout for read operations
   *
   * This parameter specifies the maximum time to wait for a read operation to end in
   * miliseconds. 
   */  
  int read_timeout;
  /**
   * \brief desired timeout for write operations
   *
   * This parameter specifies the maximum time to wait for a write operation to end in
   * miliseconds. 
   */  
  int write_timeout;
  /**
   * \brief rate at which the receive buffer is flushed
   *
   * This parameter specifies the period in miliseconds at which the receive buffer is 
   * flushed. This value can be set to any value between 2 and 255.
   */
  unsigned char latency_timer;
}TFTDIconfig;

/**
 * \brief FTDI driver class using the D2XX library
 *
 * This class implements the access to any FTDI device supported by the D2XX driver
 * provided by the manufacturer. This class inherits from the CComm class to have 
 * all the basic communications issues solved and it only has to implement the
 * specific functions to open the FTDI device (hatd_open()), config (hard_config()),
 * read from (hard_read()), write to (hard_write()) and close it (hard_close()).
 * Also it implements a function to assynchronously wait for communication events
 * to happen (hard_wait_comm_event()). 
 *
 * This class allows access to any external device that uses one of the supported
 * FTDI chips. The supported FTDI devices are:
 *
 * - FT2232H
 * - FT4232H
 * - FT232R
 * - FT245R
 * - FT2232
 * - FT232B
 * - FT245B
 * - FT8U232AM
 * - FT8U245AM
 *
 * This driver must be used when the used VID and PID values do not coincide with 
 * the dafualt values used by the manufacturer. Otherwise, it is possible to use
 * a standard serial port driver (CRS232) to access the device. However, this 
 * driver provides faster transfer rates to and from the device than the virtual
 * COM port option.
 *
 * This class uses exceptions to report errors. The name of the exception 
 * class associated to this class is CFTDIException. For a more detailes
 * descritpion, see the corresponding documentation.
 * 
 */
class CFTDI : public CComm 
{
  private:
    /**
     * \brief Handle to the FTDI device
     *
     * This handle is used to access the associated FTDI device. By default it is set
     * to NULL when the object is created, and it is initialized when the device is 
     * opened by calling the open() function. This handle remains valid until the object
     * is destroyed or the FTDI device is disconnected
     */ 
    FT_HANDLE ft_handle; 

    EVENT_HANDLE event_handle;
  protected:
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
    void hard_open(void* comm_dev);
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
    void hard_config(void *config);
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
    int hard_read(unsigned char *data, int len);
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
    int hard_write(unsigned char *data, int len);
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
    void hard_close(void);
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
    int hard_get_num_data(void);
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
    int hard_wait_comm_event(void);
  public:
    /**
     * \brief Constructor
     *
     * 
     */ 
    CFTDI(const std::string& comm_id);
    /**
     * \brief operator << overloading
     *
     * This operator allow to show important information about the FTDI device 
     * associated to the class on any ostream obejct (the standard output, a file, 
     * etc...).
     *
     * \param out A reference to an output device in which to show the desired 
     *            information.
     *
     * \param ftdi A reference to a CFTDIServer object with the information to be
     *               displayed.
     *
     * \return an object with the desired information already formatted. In this case
     *         it has the number of devices and the information on each of them.
     */
    friend std::ostream& operator<< (std::ostream& out,CFTDI& ftdi);
    ~CFTDI();
};

#endif
