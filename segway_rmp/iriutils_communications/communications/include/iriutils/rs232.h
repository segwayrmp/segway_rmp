#ifndef _RS232_H
#define _RS232_H

#include "comm.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>  
#include <fcntl.h>
#include <errno.h>
#include <string>

/**
 * \brief Available types of parity
 *
 * This enumeration provides all the possible parity types available on the
 * RS-232 serial port. The parity types are:
 *
 * * none: no parity bit is used and there is no way to detect errors.
 * * even: adds an additional bit such that the number of ones in each packet
 *         is even.
 * * odd: adds an additional bit such that the number of ones in each packet
 *        is odd.
 * * mark: adds an additional bit with the fixed value of one.
 * * space: adds an additional bit with the fixed value of zero.
 */
typedef enum {none,even,odd,mark,space} parity_type;

typedef enum {rs232_dsr=TIOCM_DSR,
              rs232_dtr=TIOCM_DTR,
              rs232_rts=TIOCM_RTS,
              rs232_cts=TIOCM_CTS,
              rs232_cd=TIOCM_CD,
              rs232_ri=TIOCM_RI} control_signals;

/**
 * \brief Configuration structure for the serial port
 */
typedef struct{
  /**
   * \brief Serial port baudrate
   *
   * This filed represents the desired speed in bits per second. Not all values are 
   * valid. The valid values for this parameter are listed below:
   *
   * - 50
   * - 75
   * - 110
   * - 134
   * - 150
   * - 200
   * - 300
   * - 600
   * - 1200
   * - 1800
   * - 2400
   * - 4800
   * - 9600
   * - 19200
   * - 38400
   * - 57600
   * - 115200
   *
   */
  int baud;
  /**
   * \brief Number of bits per paquet
   *
   * This field represents the number of bits of each packet. Not all values 
   * are valid. The valid values for this parameter are 5,6,7 or 8.
   */ 
  char num_bits;
  /**
   * \brief Parity type
   *
   * This field represents the desired parity type for each packet. This parameter 
   * must belong to the parity_type enumeration. See the documentation on this data
   * type for more information on the possible values of this parameter.
   */ 
  parity_type parity;
  /**
   * \brief Number of stop bits
   *
   * This field represents the desired number of stop bits per packet. The possible 
   * values for this parameter are only 1 or 2.
   */ 
  char stop_bits;
}TRS232_config;

/** 
 * \brief RS-232 Serial port driver
 *
 * This class implements a driver to use the standard RS-232 ports on any
 * computer. It inherits from the CComm class which provides the basic 
 * interface to any communication device. 
 *
 * This class overloads the open() and config() functions of the base class 
 * to suit the specific requirements of a serial port. Also, several 
 * functions are declared to configure the features of the serial port:
 * baudrate, number of bits, parity and number of stop bits.
 *
 * For a more detailed description of the behavior of this class, see the
 * documentation for the CComm base class.
 */
class CRS232 : public CComm
{
  private:
    /**
     * \brief write file descriptor of the communication device
     * 
     * This variable is the write file descriptor associated to the communication 
     * device. By default it is initialized to -1, and it is the inherited class
     * that must initialize it when actually opening the communication device by
     * calling the open() function. The read and write file descriptors can be
     * the same or different, depending on the particular communication device.
     */
    int serial_fd;
  protected:
    /**
     * \brief Function to set the desired baudrate
     *
     * This functions sets the speed of the srial port in bits per second.
     * If the given speed is not valid this function throws a CRS232Exception.
     *
     * \param baud the desired speed in bits per second. Not all values are 
     *             valid. The valid values for this parameter are listed below:
     *
     * - 50
     * - 75
     * - 110
     * - 134
     * - 150
     * - 200
     * - 300
     * - 600
     * - 1200
     * - 1800
     * - 2400
     * - 4800
     * - 9600
     * - 19200
     * - 38400
     * - 57600
     * - 115200
     *
     */  
    void set_baudrate(int baud);
    /**
     * \brief Function to set the number of bits per packet
     *  
     * This function sets the number of data bits included in each packet. If
     * the given number of bits is not valid, a CRS232Exception is thrown.
     *
     * \param num_bits The number of bits of each packet. Not all values are 
     *                 valid. The valid values for this parameter are 5,6,7 or
     *                 8.
     *
     */  
    void set_num_bits(char num_bits);
    /**
     * \brief Function to set the parity
     *
     * This function sets the desired parity (if any) for each of the packets.
     * If the given parity type is not valid, a CRS232Exception is thrown.
     *
     * \param parity the desired parity type for each packet. This parameter 
     *               must belong to the parity_type enumeration. See the
     *               documentation on this data type for more information
     *               on the possible values of this parameter.
     */  
    void set_parity(parity_type parity);
    /**
     * \brief Function to set the number of stop bits
     *
     * This function sets the desired number of stop bits for each packet. If
     * the given number of stop bits is not valid, a CRS232Exception is thrown.
     *
     * \param stop_bits the desired number of stop bits per packet. The possible 
     *                  values for this parameter are only 1 or 2.
     *
     */  
    void set_stop_bits(char stop_bits);
    /**
     * \brief Function to open an RS232 serial port
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
     * \verbatim
     * /dev/ttyS0
     * \endverbatim
     */
     virtual void hard_open(void *comm_dev);
     /**
     * \brief Function to configure the serial port
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
     * \param config a pointer to a TRS232_config structure already initialized
     *               with all teh configuration parameters of the serial port.
     */ 
     virtual void hard_config(void *config);
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
     virtual int hard_read(unsigned char *data, int len);
     /**
      * \brief Function to actually write to the device
      *
      * This function is automatically called either when the base class write()
      * function is called or when the data transmission end event is activated.
      * It must try to write the desired ammount of data to the communication 
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
     virtual int hard_write(unsigned char *data, int len);
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
      * - 0 for the end of transmission event
      * - 1 for the new data received event
      * - 2 for the error event
      */
     virtual int hard_wait_comm_event(void);
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
     virtual void hard_close(void);
  public:
    /**
     * \brief Constructor
     *
     * This constructor creates a new CRS232 object initialized by default, 
     * but it does not open any physical serial port. This constructor calls
     * the base class constructor with the serial port unique identifier
     * provided to the constructor.
     *
     * \param comm_id A null terminated string which identified the
     *                communication device. This string is used to create a
     *                unique identifier for all the threads and events of the
     *                class.
     *
     */ 
    CRS232(const std::string& comm_id);
    /**
     * \brief
     *
     */
    void set_control_signal(control_signals signal);
    /**
     * \brief
     *
     */
    void clear_control_signal(control_signals signal);
    /**
     * \brief
     *
     */
    bool get_control_signal(control_signals signal);
    /**
     * \brief destructor
     *
     * This destructor does nothing. The base class destructor is the one in 
     * charge of freeing all the allocated resources.
     */  
    ~CRS232();
};

#endif
