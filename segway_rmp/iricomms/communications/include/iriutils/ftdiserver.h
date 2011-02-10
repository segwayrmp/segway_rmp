#ifndef _FTDI_SERVER
#define _FTDI_SERVER

#include <vector>
#include <iostream>
#include "ftdimodule.h"
#include "mutex.h"

const int FTDI_VID=0x0403;
const int DEFAULT_FTDI_PID[]={0x6001,0x6010,0x6006};

/**
 * \brief structure with information about a device
 *
 * This structure holds information about one of the devices available on a system.
 * This strcucture is first initialized when the device is first detected, and then
 * it is stored in the internal device list.
 */
typedef struct
{
  /**
   * \brief opened flag
   *
   * This flag indicates if the USB device is opened by the system or any other
   * application. It will anly be possible to use devices that are not currently
   * opened by any othe process. Trying to do so will throw an exception.
   */ 
  bool opened;
  /**
   * \brief High speed flag
   *
   * This flag indicates if the corresponding device supports the high speed 
   * specification of the USB (true) or not (false).
   */ 
  bool high_speed;
  /**
   * \brief Type of the USB device
   */  
  unsigned long int type;
  /**
   * \brief USB device identification
   */  
  unsigned long int id;
  /**
   * \brief location of the device on the bus
   */
  unsigned long int location;
  /**
   * \brief Serial number of the device
   */ 
  std::string serial_number;
  /**
   * \brief Decription of the device
   */  
  std::string description;
}TDevice_info;

/**
 * \brief Global event server
 *
 * This class implements an FTDI device server which is global to the application
 * and also only one instance exist that is shared by all obects requiring 
 * access to an FTDI device.
 *
 * Each FTDI device is identified by a VID and PID combination. At construction
 * time this server searches the system for devices with the default VID and PID
 * combinations provided by the manufacturer. If there exist devices with 
 * different VID and PID combinations it is necessary to call the add_custom_PID()
 * function, which rescans the system for available devices and adds them into
 * the internal list.
 *
 * The get_num_devices() function can be used to retrieve the number of FTDI
 * devices available on the system. To actually get one CFTDI class object to
 * handle the desired device it is necessary to call the get_device() function,
 * either providing the location of the desired device, the serial number or
 * its description. It will be impossible to get an object of a device that it 
 * is already in use by the system or any other application. 
 *
 * This class uses exceptions to report errors. The name of the exception 
 * class associated to this class is CFTDIServerException. For a more detailes
 * descritpion, see the corresponding documentation.
 *
 */
class CFTDIServer
{
  private:
    /**
     * \brief Reference to the unique instance of this class
     *
     * This reference points to the unique instance of this class. By default 
     * it is set to NULL, and it is initialized in the first call to the 
     * instance() function. after that, successive calls to rhat function
     * will return the pointer to the previously created object.
     */ 
    static CFTDIServer* pinstance;
    /**
     * \brief Information on all FTDI devices available
     *
     * This list have important information on all FTDI devices available on
     * the system with one of the default or custom PID values. The information
     * include:
     *
     * - Opened flag (as a boolean)
     * - High speed flag (as a boolean) 
     * - Device type (as an unsigned long int)
     * - Device id (as an unisgned long int)
     * - Device location (as an int)
     * - Serial number (as a string)
     * - Description (as a string)
     *
     * This list if first initialized at construction time with the devices
     * presnt with the default PID values. When new PID values are added and
     * FTDI devices with those PID values are present, the list is expanded
     * wiht the information of the new devices.
     *
     */ 
    std::vector<TDevice_info> devices;
    /**
     * \brief List of all custom PID values
     *
     * This list has all the PID added by the user. Initially it is empty,
     * and it is updated any time the add_custom_PID() function is called.
     */  
    std::vector<int> custom_PID;
  protected:
    /**
     * \brief Default constructor
     *
     * This constructor initializes the available FTDI devices list with the 
     * default VID and PID combinations. This list can only be modified when
     * a new PID is added and the sustem is rescaned for new devices.
     *
     * The reference to the newly created object is not modified. This constructor 
     * is only called once and from inside the instance() function. It can not
     * be called directly by the user since it is declared as protected.
     *
     */ 
    CFTDIServer();
    /** 
     * \brief Copy constructor
     *
     * This constructor is used to initialize a new object with the contents of
     * an existing one. Since there could be only one instance of this class,
     * only the pinstance attribute must be copied, but since it is static
     * nothing is to be done in this constructor.
     *
     * \param object an existing instance of a CFTDIServer class which has been
     *               already initialized.
     *
     */ 
    CFTDIServer(const CFTDIServer& object);
    /**
     * \brief assign operator overloading
     *
     * This function overloads the assign operator for this class. Since there 
     * could be only one instance of this class, only the pinstance attribute 
     * must be copied, but since it is static, nothing is to be done.
     *
     * \param object an existing instance of a CFTDIServer class which has been
     *               already initialized.
     */ 
    CFTDIServer& operator = (const CFTDIServer& object);
  public:
    /**
     * \brief Function to get a reference to the unique instance
     *
     * This function returns a reference to the only instance of the singleton.
     * When it is first called, a new object of this class is created, and the
     * successive calls only return a reference to the previously created
     * object.
     *
     * Since this function is static, it can be call anywhere in the code so
     * all objects can access the unique event handler. 
     *
     * \return A reference to the only instance of this class. This reference 
     *         must not be freed until the application ends. 
     *  
     */ 
    static CFTDIServer* instance(void);
    /**
     * \brief Function to add new PID values
     *
     * This function can be used to include new VID and PID combinations. By
     * default the FTDI driver only recognises devices with the default PID
     * values. Some manufacturers provide their own PID, so it is necessary to 
     * include it using this function.
     *
     * When this function is called, the system is rescaned to find any available
     * FTDI device with the provided PID. In case there is any, the internal 
     * device list is expanded with the new information. If there is no device
     * with the provided PID this function does nothing.
     *
     * This class uses exceptions to report errors. The name of the exception 
     * class associated to this class is CFTDIServerException. For a more detailes
     * descritpion, see the corresponding documentation.
     *
     * \param PID a positive value corresponding to the desired PID value. This
     *            value must be the one provided by the manufacturer.
     */ 
    void add_custom_PID(int PID);
    /**
     * \brief Function to get the number of available FTDI devices
     *
     * This function return to total number of FTDI devices present in the system
     * with any of the default or custom PID values. Some of the devices may be
     * used by the system or other applications. In this case, they can not be 
     * accessed, but they are listed anyway.
     *
     * This class uses exceptions to report errors. The name of the exception 
     * class associated to this class is CFTDIServerException. For a more detailes
     * descritpion, see the corresponding documentation.
     *
     * \return The number of devices available on the system
     */ 
    int get_num_devices(void);
    /**
     * \brief Function to check if teh device is opened
     *
     * This function returna wether the device located at the given position of the
     * internal list is opened or not.
     *
     * \param index is an integer with the position on the internal list of the 
     *              desired device. This value must be between 0 and the number of
     *              devices on the list -1.
     *
     * \return a boolean that indicates if the device is opened (true) or not (false).
     */ 
    bool is_opened(int index);
    /**
     * \brief Function to check the maximum speed supported
     *
     * This function returns wether the device located at the given position of the
     * internal list supports high speed modes or not.
     *
     * \param index is an integer with the position on the internal list of the 
     *              desired device. This value must be between 0 and the number of
     *              devices on the list -1.
     *
     * \return a boolean that indicates if the device supports high speed modes (true)
     *         or not (false).
     */ 
    bool is_high_speed(int index);
    /**
     * \brief Function to get the type of the device
     *
     * This function returns the type of the USB device located at the given position
     * of the internal list.
     *
     * \param index is an integer with the position on the internal list of the 
     *              desired device. This value must be between 0 and the number of
     *              devices on the list -1.
     * 
     * \return an integer which represents the type of the USB device.
     */ 
    unsigned long int get_type(int index);
    /**
     * \brief Function to get the device identifier
     *
     * This function returns the identifier of the USB device located at the given 
     * position of the internal list.
     *
     * \param index is an integer with the position on the internal list of the 
     *              desired device. This value must be between 0 and the number of
     *              devices on the list -1.
     * 
     * \return an integer which represents the identifier of the USB device.
     *
     */ 
    unsigned long int get_id(int index);
    /**
     * \brief Function to get the location of the device on teh bus
     *
     * This function returns the location of the USB device on the USB bus, which is 
     * located at the given position of the internal list.
     *
     * \param index is an integer with the position on the internal list of the 
     *              desired device. This value must be between 0 and the number of
     *              devices on the list -1.
     * 
     * \return an integer which represents the location of the USB device on the bus.
     */ 
    unsigned long int get_location(int index);
    /**
     * \brief Function to get the serial number of the device
     *
     * This function return the serial number of the USB device located at the given 
     * position of the internal list as a string.
     *
     * \param index is an integer with the position on the internal list of the 
     *              desired device. This value must be between 0 and the number of
     *              devices on the list -1.
     * 
     * \return a string with the serial number of the USB device.
     */
    std::string& get_serial_number(int index);
    /**
     * \brief Function to get the description of the device
     *
     * This function return the description of the USB device located at the given 
     * position of the internal list as a string.
     *
     * \param index is an integer with the position on the internal list of the 
     *              desired device. This value must be between 0 and the number of
     *              devices on the list -1.
     * 
     * \return a string with the description of the USB device.
     */ 
    std::string& get_description(int index);
    /**
     * \brief Function to get a new CFTDI object
     *
     * Function to get a CFTDI class object associated to the desired FTDI device.
     * In this case the desired FTDI device is identified by either its serial number
     * or its description. The serial number is in general the best way to select
     * the desired device since it is unique. 
     *
     * However, the description may be shared by several devices of the same kind. 
     * In this second case, the first device found on the list is returned.
     * If the desired device is already being used by the system or any other 
     * application this function will fail throwing an exception of the
     * CFTDIServer class. Otherwise, a new CFTDI object is returned. 
     *
     * The CFTDI object returned by this function is already opened using the
     * serial number or description provided to the function. The name of the 
     * returned object is created by appending the provided serial number or
     * description to "FTDI_" to have a unique identifier for each device.
     *
     * This class uses exceptions to report errors. The name of the exception 
     * class associated to this class is CFTDIServerException. For a more detailes
     * descritpion, see the corresponding documentation.
     *
     * \param serial_desc a string with either the serial number or the description
     *                    of the desired FTDI device.
     *
     * \return on success this function returns a pointer a newly created CFTDI
     *         object associated to the desired FTDI device. The calling process
     *         is responsible for destroying the returned obejct.
     */ 
    CFTDI *get_device(std::string& serial_desc);
    /**
     * \brief operator << overloading
     *
     * This operator allow to show important information about the class and the
     * information of all FTDI devices available on any ostream obejct (the standard
     * output, a file, etc...).
     *
     * \param out A reference to an output device in which to show the desired 
     *            information.
     *
     * \param server A reference to a CFTDIServer object with the information to be
     *               displayed.
     *
     * \return an object with the desired information already formatted. In this case
     *         it has the number of devices and the information on each of them.
     */ 
    friend std::ostream& operator<< (std::ostream &out,CFTDIServer &server);
};

#endif
