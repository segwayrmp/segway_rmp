#ifndef _SEGWAYRMP200_DRIVER_H
#define _SEGWAYRMP200_DRIVER_H

#include "ftdimodule.h"
#include "ftdiserver.h"
#include "mutex.h"
#include <string>

const short int SEGWAY_PID = 0xE729;

/**
 * \brief structure of a USB packet 
 *
 * This new data type has all the information of a data packet send to or received
 * from the USB communication device. This data type is only 18 unsigned char values 
 * arranged as a vector.
 *
 */
typedef struct 
{
  /**
   * \brief packet data
   *
   * This vector has all the data of a packet that has been received or that is going
   * to be sent. The format of this packet is as follows:
   *
   */  
  unsigned char data[18];
}segway_packet;

/**
 * \brief Valid gain schedules for the segway
 *
 * This new data type enumerates all the possible gain schedules of a segway platform.
 * This data type is only used on the balancing mode and ignored in the tractor mode.
 * The possible gain schedule values are:
 *
 * - light: for relativelly small weights (20 kg) placed near the platform.
 * - tall: for relativelly small weights (20 kg) distributed along the vertical axis
 *         of the robot.
 * - heavy: for heavy weights (40 kg) placed near the platform.
 *
 */
typedef enum {light,tall,heavy} gain;

/**
 * \brief Valid operation modes of the segway
 *
 * This new data type enumerates all the possible operation modes of a segway platform.
 * The possible operation modes are:
 *
 * - tractor: in this mode the balancing feature is disabled and the platform needs an
 *            additional support point (castor wheel). In this mode it is possible to
 *            command the robot to move forward/reverse and also turn.
 * - balance: in this mode the segway balances itself, trying to keep the given position.
 *            In this mode it is possible to move the robot without a castor wheel.
 * - power down: the platform is shut down and no power is provided to the motors. It is 
 *               possible to enter this mode from either the tractor and balance modes.
 *
 */
typedef enum {tractor=1,balance=2,power_down=3} op_mode;

/**
 * \brief Segway RMP 200 driver
 *
 * This class implements the basic interface to communcate with an RMP200 segway platform,
 * configure it and also send motion commands. By default the communication with the 
 * hardware platform is achieved through USB.
 *
 * After construction, the first thing to do is assign the new segway object with the 
 * communication device to be used using the set_comm_dev() function. Before that, the
 * communication with the device is not possible and any attempt will result in an exception
 * being thrown. When the communication device is set, the two main threads of the class
 * start. These threads are:
 *
 * - The command thread: which is responsible of sending the mnotion command to the platform
 *                       at regular intervals. Currently the update rate for the motion 
 *                       command is fixed to 50 timer per second. 
 *
 *                       When the user gives a motion command using the move() or stop() 
 *                       functions, the new command is not immediatelly send to the robot,
 *                       but stored into internal parameters of the class. It is this thread
 *                       that sends the new commands to the robot at regular intervals.
 *
 *                       At the moment it is not possible to change the update rate of the 
 *                       motion commands.
 *
 * - The feedback thread: this thread is responsible of reding all the data sent by the 
 *                        robot platform and store it in the internal parameters of the 
 *                        class as fast as they are sent.
 *
 *                        When the user tries to read the value of one of the parameters 
 *                        of the platform, the returned value is the one stored inside 
 *                        the class, which are updated by this thread.
 *
 *                        At the moment only data messages from 2 to 7 are handled, as well
 *                        as the heart beat. The other messages are received but ignored.
 *
 * The public interface of this class alows to completelly configure all the parameters of
 * the segway platform as well as monitor each of the available feedback parameters. Also
 * there exist an operator << to show all feedback information already formated.
 *
 */
class CSegwayRMP200
{
  private:
    /**
     * \brief unique identifier of the segway platform
     *
     * This string has the unique identifier of the segway platform. This string is
     * initialized at construction time and can not be modified afterwards. The 
     * get_segway_id() function can be used to get this string. 
     *
     * This string is also used to create unique identifier for all the threads and
     * events used inside the class.
     *
     */
    std::string id;	
    /**
     * \brief mutex for the status data
     *
     * This mutex is used to control the access to the status variables (shared memory)
     * of the class. These variables are periodically updated by the feedback thread 
     * and they can be read by the user at any time using the get functions. 
     *
     * This mutex is used by the feedback thread and all the get status function to 
     * avoid data corruption while simultaneously reading and writing to the status 
     * variable of the segway platform. There exist a different mutex for the command 
     * data.
     *
     */
    CMutex access_status;
    /**
     * \brief mutex for the command data
     *
     * This mutex is used to control the access to the command variables (shared memory)
     * of the class. These variables are changed by the user at any time using the mov()
     * or stop() functions, and the new command is actually send to the segway platform
     * periodically by the command thread. 
     *
     * This mutex is used by the command thread and all the motion function to avoid data
     * corruption while simultaneously reading and writing to the command variable of the
     * segway platform. There exist a different mutex for the status data.
     */
    CMutex access_command;
    /**
     * \brief a reference to the FTDI USB device
     *
     * This attribute points to the communication device used to send and receive data 
     * to and from the segway platform. The communication device is cretaed inside the 
     * object at initialization time using the CFTDIServer and the description or serial
     * number of the desired platform via the connect() function.
     *
     * It is not possible to send or receive data to or from the platform until the 
     * driver is not connected to the hardware platform. Any attempt to do so will 
     * result in an exception being thrown.
     *
     */
    CFTDI *comm_dev;
    /**
     * \brief reference to the unique ftdi_server
     *
     * This reference to the unique ftdi server is initialized when an object of
     * this class is first created. It is used to create and handle all the
     * FTDI device drivers used to access the segway platform. The object pointed 
     * by this reference is shared by all objects in any application.
     *
     */
    CFTDIServer *ftdi_server; 
    /**
     * \brief Reference to the unique event handler
     *
     * This reference to the unique event handler is initialized when an object of
     * this class is first created. It is used to create and handle all the
     * segway platform events. The object pointed by this reference is shared by all
     * objects in any application.
     */
    CEventServer *event_server;
    /**
     * \brief Reference to the unique thread handler
     *
     * This reference to the unique thread handler is initialized when an object of
     * this class is first created. It is used to create and handle all the
     * segway platform threads. The object pointed by this reference is shared by all
     * objects in any application.
     */
    CThreadServer *thread_server;
    /**
     * \brief identifier of the feedback thread
     *
     * This string has the identifier of the thread used to read status information from
     * the segway platform. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_read_thread". This
     * thread is only used internally to the class, so it is not possible to get its
     * identifier out.
     */
    std::string read_thread_id;
    /**
     * \brief identifer of the command thread
     *
     * This string has the identifier of the thread used to send motion commands to 
     * the segway platform. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_command_thread". This
     * thread is only used internally to the class, so it is not possible to get its
     * identifier out.
     */
    std::string command_thread_id;
    /**
     * \brief identifier of the reception event of the communication device
     * 
     * This string has the identifier of the event generated by the communcation device 
     * to signal de reception of new data. This event is not created by this class, but
     * retrieved from the communication device when it is attached to this class. This
     * event is used by the read thread to awake when new data is received. 
     */
    std::string comm_rx_event;
    /**
     * \brief identifier of the event to finish the feedback thread
     *
     * This string has the identifier of the event used to signal the feedback thread
     * to end its execution. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_finish_read_thread".
     * This event is only used inside the class, so it is not possible to get its
     * identifier out.
     */
    std::string read_finish_event;
    /**
     * \brief identifier of the event to finish the command thread
     *
     * This string has the identifier of the event used to signal the command thread
     * to end its execution. This string is initialized at contruction time by appending
     * the unique identifier of the segway object and the string "_finish_command_thread".
     * This event is only used inside the class, so it is not possible to get its
     * identifier out.
     */
    std::string command_finish_event;
  protected:
    // status variables
    /**
     * \brief left wheel velocity
     *
     * This value has the velocity of the left wheel in degrees per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_left_wheel_velocity() function or the
     * overloaded operator <<.
     */
    float left_wheel_velocity;
    /**
     * \brief right wheel velovity
     *
     * This value has the velocity of the right wheel in degrees per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_right_wheel_velocity() function or the
     * overloaded operator <<.
     */
    float right_wheel_velocity;
    /**
     * \brief pitch angle
     *
     * This value has the pitch angle of the platform in degrees. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_pitch_angle() function or the
     * overloaded operator <<.
     */
    float pitch_angle;
    /**
     * \brief pitch rate
     *
     * This value has the pitch rate of the platform in degrees per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_pitch_rate() function or the
     * overloaded operator <<.
     */
    float pitch_rate;
    /**
     * \brief roll angle
     *
     * This value has the roll angle of the platform in degrees. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_roll_angle() function or the
     * overloaded operator <<.
     */
    float roll_angle;
    /**
     * \brief roll rate
     *
     * This value has the roll rate of the platform in degrees per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_roll_rate() function or the
     * overloaded operator <<.
     */
    float roll_rate;
    /**
     * \brief yaw rate
     *
     * This value has the yaw rate of the platform in degrees per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_yaw_rate() function or the
     * overloaded operator <<.
     */
    float yaw_rate;
    /**
     * \brief number of frames per second
     *
     * This value has the number of servo frames in fames per second. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_servo_frames() function or the
     * overloaded operator <<.
     */
    float servo_frames; 
    /**
     * \brief left wheel displacement
     *
     * This value has the displacement of the left wheel in meters. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_left_wheel_displacement() function 
     * or the overloaded operator <<.
     */
    float left_wheel_displ;
    /**
     * \brief right wheel displacement
     *
     * This value has the displacement of the right wheel in meters. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_right_wheel_displacement() function 
     * or the overloaded operator <<.
     */
    float right_wheel_displ;
    /**
     * \brief forward displacement
     *
     * This value has the displacement of the whole platform in meters. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_forward_displacement() function 
     * or the overloaded operator <<.
     */
    float forward_displ;
    /**
     * \brief yaw displacement
     *
     * This value has the rotation of the whole platform in revolutions. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_yaw_displacement() function 
     * or the overloaded operator <<.
     */
    float yaw_displ;
    /**
     * \brief left motor torque
     *
     * This value has the torque of the left motor in Newtorn per meter. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_left_motor_torque() function 
     * or the overloaded operator <<.
     */
    float left_torque;
    /**
     * \brief right motor torque
     *
     * This value has the torque of the right motor in Newtorn per meter. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_right_motor_torque() function 
     * or the overloaded operator <<.
     */
    float right_torque;
    /**
     * \brief operation mode
     *
     * This value has the current operation mode of the platform. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_operation_mode() function 
     * or the overloaded operator <<. The possible operation modes are:
     *
     * - tractor
     * - balance
     * - power down
     *
     * See the documentation of the op_mode data type for more information on the
     * possible operation modes.
     */
    op_mode mode;
    /**
     * \brief gain schedule
     *
     * This value has the current gain schedule of the platform. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_gain_schedule() function 
     * or the overloaded operator <<. The possible gain schedules are:
     *
     * - light
     * - tall
     * - heavy.
     *
     * See the documentation of the gain data type for more information on the 
     * possible gain schedule.
     */
    gain gain_schedule;
    /**
     * \brief user battery
     *
     * This value has the voltage of the UI battery in Volts. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_ui_battery_voltage() function 
     * or the overloaded operator <<.
     */
    float ui_battery;
    /**
     * \brief power base battery
     *
     * This value has the voltage of the power base battery in Volts. This value is
     * periodically updated by the read thread and it is not possible to modify it 
     * otherwise. To get its value use the get_powerbase_battery_voltage() function 
     * or the overloaded operator <<.
     */
    float powerbase_battery;
    // command variables
    /**
     * \brief translation velocity
     * 
     * This value has the desired translational velocity in meters per second. This 
     * values is updated each time the move() or stop() functions are called. Then
     * this value is sent periodically to the segway platform by the command thread.
     */
    short int vT; 
    /**
     * \brief rotation velocity
     *
     * This value has the desired rotational velocity in revolutions per second. This 
     * values is updated each time the move() or stop() functions are called. Then
     * this value is sent periodically to the segway platform by the command thread.
     */
    short int vR; 
    // methods
    /**
     * \brief function to compute checksum of USB packets 
     *
     * This function computes the checksum of a given USB packet either received from
     * the platform or to be send to it. When a new packet is received this function 
     * is used to check that the packet has no errors. Since the packet is sent with
     * a checksum, the return value of this function should be 0.
     *
     * When a packet is about to be sent, this function is used to compute the 
     * checksum value for the packet. The returned value then is used to complete
     * the data packet (18th bytes) before it is sent to the segway platform.
     *
     * This function is only used internally by the feedback and command threads,
     * but it can not be used elsewhere.
     *
     * \param packet a reference to the packet on which to compute the chcksum. If
     *               the packet has been received it should already has a checksum
     *               value at the 18th position, and otherwise this position should
     *               be set to 0 to correctly compute the checksum.
     *
     * \return the value of the checksum. If the packet has been received, this value 
     *         should be 0 for a valid packet and any other value otherwise. If the 
     *         packet is to be sent, this value is the checksum to be used to complete
     *         the packet.
     */		
    unsigned char compute_checksum(segway_packet *packet);
    /**
     * \brief function to read a whole packet from the segway platform
     *
     * This functions is used to read full packets from the segway platform. It first 
     * syncronizes the incomming data to detect the packet header, and then stores all
     * the data of the packet in the given packet strcuture. 
     *
     * This function reads all available data and returns immediatelly even if the 
     * whole packet has not been received. If several packets have been received, this
     * function returns after reading the first one, and the function has to be recalled
     * until no more data is available. This function is only used internally by the 
     * feedback thread.
     *
     * \param packet a reference to a packet structure in which to store the received 
     *               packet. The necessary memory for this structure has to be allocated
     *               before calling this function. This contents of this structure are
     *               only valid when the return value of this function is true, otherwise
     *               its contents should be ignored, but not modified since it has 
     *               partial packet information.
     *
     * \return a flag that indicated if a whole packet has been read (true) or not (false).
     *         only when the return value is true, the contents of the packet parameter
     *         can be used.
     */
    bool read_packet(segway_packet *packet,int *packet_len);
    /**
     * \brief function to parse whole packets read from the segway platform
     *
     * This function is used to extract the data contained into the data packets received.
     * The checksum is first checked and if there is any error, an exception is thrown.
     * Otherwise, the data from the packet is stored into the corresponding internal 
     * attributes of the class.
     *
     * This function blocks the status mutex to avoid errors while updating the internal
     * attributes. This way all functions to get the current status will get blocked until
     * the update is complete.
     *
     * \param packet a reference to a packet structure which stores the data of a new 
     *               received packet. The packet passed to this function should be the
     *               one returned by the read_packet() function. 
     *
     */
    void parse_packet(segway_packet *packet);
    /**
     * \brief Thread function that reads data from segway
     *
     * This functions waits for any data to be received from the segway platform. When a whole
     * packet is read from the robot, it is parsed and then the internal information of the class
     * updated.
     *
     * This thread is created at construction time but it is not started until the communication
     * device is attached to avoid errors. This thread is always active until the finish event is
     * activated, which happens when the close() function is called.
     *
     * \param param a pointer to the object itself. since this function is statis, there exist no
     *              this pointer, so this parameter is used to access the internal attributes of
     *              the class.
     * 
     * \return in this case, the function always return NULL, so no data is returned to the 
     *         main thread.
     */
    void read_thread(void);
    /**
     * \brief
     *
     */
    static void *start_read_thread(void *param); 
    /**
     * \brief Thread function that sends commands to the segway
     *
     * This function is used to send motion commands to the segway platform periodically. This 
     * functions gets the internal motion commands (translational and rotational velocities) from
     * the class and sends it to the robot every 20ms. At this moment it is not possible to
     * change the update drate of the motion commands.
     *
     * This thread is created at construction time but it is not started until the communication
     * device is attached to avoid errors. This thread is always active until the finish event is
     * activated, which happens when the close() function is called. If this thread ends unexpectly
     * the robot will stop after a few moments, since it requires constant commands from the host
     * computer to continued operation.
     *
     * To change the current motion command, use the move() function. The stop() function should
     * be used to immediatelly stop the motors. These functions will modify the internal attributes
     * of the class, and this thread will be the one responsible of sending the new commands to
     * the robot.
     *
     * \param param a pointer to the object itself. since this function is statis, there exist no
     *              this pointer, so this parameter is used to access the internal attributes of
     *              the class.
     * 
     * \return in this case, the function always return NULL, so no data is returned to the 
     *         main thread.
     *
     */
    void command_thread(void);
    /**
     * \brief
     *
     */
    static void *start_command_thread(void *param); 		
  public:
    /**
     * \brief constructor
     *
     * This constructor creates and initializes all the threads and events nedded by the class.
     * However, the threads are not started until the set_comm_dev() is called. This constructor
     * accepts a string with the unique identifier of the segway platform, which is used to 
     * create the unique identifiers of both events and threads.
     *
     * \param segway_id a string with the unique identifier of the segway platform. This string
     *                  must be not empty, and it is used to also create the identifiers of both
     *                  events and threads.
     */
    CSegwayRMP200(std::string& segway_id);
    /**
     * \brief function to get the identifeir of the segway
     *
     * This function is used to get the unique identifier of the segway platform associated
     * to the driver.
     *
     * \return a null terminated string with the identifier of the segway.
     */
    std::string& get_id(void);
    // configuration functions
    /**
     * \brief function to set the velocity scale factor
     *
     * This function is used to set the desired scale factor for the velocity. This scale factor 
     * is used to change the maximum velocity allowed. The given motion commands are multiplied
     * by this scale factor, thus limiting the maximum possible speed. Contrary to what happens
     * with the motion commands, this function immediatelly sends a data packet to the segway 
     * platform.
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     *
     * \param factor the desired scale factor for the velocity. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception.
     */
    void set_velocity_scale_factor(float factor);
    /**
     * \brief function to set the acceleration scale factor
     *
     * This function is used to set the desired scale factor for the acceleration. This scale factor 
     * is used to change the maximum acceleration allowed. The given motion commands are multiplied
     * by this scale factor, thus limiting the maximum possible acceleration. Contrary to what 
     * happens with the motion commands, this function immediatelly sends a data packet to the 
     * segway platform.
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     *
     * \param factor the desired scale factor for the acceleration. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception.
     */
    void set_acceleration_scale_factor(float factor);
    /**
     * \brief function to set the turnrate scale factor
     *
     * This function is used to set the desired scale factor for the turn rate. This scale factor 
     * is used to change the maximum turn rate allowed. The given motion commands are multiplied
     * by this scale factor, thus limiting the maximum possible turn speed. Contrary to what happens
     * with the motion commands, this function immediatelly sends a data packet to the seigway platform.
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     *
     * \param factor the desired scale factor for the turn rate. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception
     */
    void set_turnrate_scale_factor(float factor);
    /**
     * \brief function to set the gain schedule
     *
     * This function is used to change the current gain schedule of the segway platform. The gain
     * schedule is used to improve the balance capabilities of the platform. Depending on the 
     * weight of the payload and on its distribution, this parameter must be changed. Contrary to
     * what happens with the motion commands, this function immediatelly sends a data packet to 
     * the segway platform.
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     *
     * \param value the desired value for the gain schedule. This parameter should be one of the
     *              following values:
     *
     *              - light
     *              - tall
     *              - heavy
     *
     *              See the documentation on the gain data type for more information on the 
     *              different meaning of the gain schedules.
     */
    void set_gain_schedule(gain value);
    /**
     * \brief function to set the current limit scale factor
     *
     * This function is used to set the desired scale factor for the current limit. This scale 
     * factor is used to change the maximum motor current allowed. Contrary to what happens with
     * the motion commands, this function immediatelly sends a data packet to the segway platform.
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     *
     * \param factor the desired scale factor for the current limit. The possible values for this 
     *               parameter must be limited between 0.0 and 1.0. Any other value is not 
     *               allowed and will throw an exception
     */
    void set_currentlimit_scale_factor(float factor);
    /**
     * \brief function to lock the balance function
     *
     * This function is used to block the balance mode. 
     */
    void lock_balance(void);
    /**
     * \brief function to unlock the balance function
     *
     * This function is used to unblock the balance mode. 
     */
    void unlock_balance(void);
    /**
     * \brief function to set the operation mode
     *
     * This function is used to change the current operation mode of the segway platform. The 
     * operation mode is used to change the working mode of the segway platform. Contrary to
     * what happens with the motion commands, this function immediatelly sends a data packet to 
     * the segway platform.
     *
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     *
     * \param mode the desired value for the operation mode. This parameter should be one of the
     *             following values:
     *
     *             - tractor
     *             - balance
     *             - power down
     *
     *             See the documentation on the op_mode data type for more information on the 
     *             different meaning of the operation modes.
     */
    void set_operation_mode(op_mode mode);
    /**
     * \brief function to reset the right wheel integrator
     *
     * This function is used to reeet the right wheel integrator of the segway platform. This 
     * integrator holds the total distance traveled by the right wheel. Contrary to what happens with
     * the motion commands, this function immediatelly sends a data packet to the segway platform.
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     *     
     */
    void reset_right_wheel_integrator(void);
    /**
     * \brief function to reset the left wheel integrator
     *
     * This function is used to reset the left wheel integrator of the segway platform. This 
     * integrator holds the total distance traveled by the left wheel. Contrary to what happens with
     * the motion commands, this function immediatelly sends a data packet to the segway platform.
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     */
    void reset_left_wheel_integrator(void);
    /**
     * \brief function to reset the yaw integrator
     *
     * This function is used to reset the yaw integrator of the segway platform. This 
     * integrator holds the total rotation performed by the robot. Contrary to what happens with
     * the motion commands, this function immediatelly sends a data packet to the segway platform.
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     */
    void reset_yaw_integrator(void);
    /**
     * \brief function to reset the forward integrator
     *
     * This function is used to reset the forward integrator of the segway platform. This 
     * integrator holds the total forward displacement of the robot. Contrary to what happens with
     * the motion commands, this function immediatelly sends a data packet to the segway platform.
     * 
     * This function can be called at any time, but it is recomended to use it only once at the
     * configuration stage of the robot, before sending motion commands to it. This function 
     * throws a CSegwayRMP200Exception to report errors.
     */
    void reset_forward_integrator(void);
    /**
     * \brief function to connect to the hardware platform
     *
     * This function is used to connect the driver with the hardware platform. The
     * communication device is only created and initialized when this function is 
     * called, so it is not possible to send or receive data to or from the platform
     * until then. When this function is called, both the command and feedback threads 
     * are started. Otherwise, the communication is not possible.
     *
     * This function throws a CSegwayRMP200Exception exception to report any error.
     *
     * \param desc_serial a null terminated string with the description or the serial
     *                    number of the segway platform to associate to the driver.
     *                    This string can be obtanied through the CFTDIServer or else
     *                    hardcoded if known.
     *
     */
    void connect(std::string& desc_serial);
    // status functions
    /**
     * \brief function to return the pitch angle
     *
     * This function returns the current pitch angle in degrees. This function 
     * only returns the value of the internal attribute, but it does not access 
     * the hardware platform. This value is periodically updated by the feedback 
     * thread.
     *
     * \return the current pitch angle in degrees.
     */
    float get_pitch_angle(void);
    /**
     * \brief function to return the pitch rate
     *
     * This function returns the current pitch rate in degrees per second. This 
     * function only returns the value of the internal attribute, but it does not 
     * access the hardware platform. This value is periodically updated by the 
     * feedback thread.
     *
     * \return the current pitch rate in degrees per second.
     */
    float get_pitch_rate(void);
    /**
     * \brief function to return the roll angle
     *
     * This function returns the current roll angle in degrees. This function 
     * only returns the value of the internal attribute, but it does not access 
     * the hardware platform. This value is periodically updated by the feedback 
     * thread.
     *
     * \return the current roll angle in degrees.
     */
    float get_roll_angle(void);
    /**
     * \brief function to return the roll rate
     *
     * This function returns the current roll rate in degrees per second. This 
     * function only returns the value of the internal attribute, but it does not 
     * access the hardware platform. This value is periodically updated by the 
     * feedback thread.
     *
     * \return the current roll rate in degrees per second.
     */
    float get_roll_rate(void);
    /**
     * \brief function to return the left wheel velocity
     *
     * This function returns the current left wheel velocity in meters per second. 
     * This function only returns the value of the internal attribute, but it 
     * does not access the hardware platform. This value is periodically updated 
     * by the feedback thread.
     *
     * \return the current left wheel in meters per second.
     */
    float get_left_wheel_velocity(void);
    /**
     * \brief function to return the right wheel velocity
     *
     * This function returns the current right wheel velocity in meters per second. 
     * This function only returns the value of the internal attribute, but it 
     * does not access the hardware platform. This value is periodically updated 
     * by the feedback thread.
     *
     * \return the current right wheel in meters per second.
     */
    float get_right_wheel_velocity(void);
    /**
     * \brief function to return the yaw rate
     *
     * This function returns the current yaw rate in revolutions per second. 
     * This function only returns the value of the internal attribute, but it 
     * does not access the hardware platform. This value is periodically updated 
     * by the feedback thread.
     *
     * \return the current yaw rate in revolutions per second.
     *
     */
    float get_yaw_rate(void);
    /**
     * \brief function to return the number of servo frames per second
     *
     * This function returns the current number of servo frames per second. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current number of servo frames per second.
     */
    float get_servo_frames(void);
    /**
     * \brief function to return the left wheel displacement
     *
     * This function returns the current left wheel displacement in meters. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current left wheel displacement in meters.
     */
    float get_left_wheel_displacement(void);
    /**
     * \brief function to return the right wheel displacement
     *
     * This function returns the current right wheel displacement in meters. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current right wheel displacement in meters.
     */
    float get_right_wheel_displacement(void);
    /**
     * \brief function to return the total forward displacement
     *
     * This function returns the current forward displacement in meters. This 
     * function only returns the value of the internal attribute, but it does
     * not access the hardware platform. This value is periodically updated by
     * the feedback thread.
     *
     * \return the current forward displacement in meters.
     *
     */
    float get_forward_displacement(void);
    /**
     * \brief function to return the total yaw displacement
     *
     * This function returns the current yaw displacement in revolutions per 
     * second. This function only returns the value of the internal attribute,
     * but it does not access the hardware platform. This value is periodically 
     * updated by the feedback thread.
     *
     * \return the current yaw displacement in revolutions per second.
     *
     */
    float get_yaw_displacement(void);
    /**
     * \brief function to return the current left motot torque
     *
     * This function returns the current left motor torque in Nm. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current left motor torque in Nm.
     */
    float get_left_motor_torque(void);
    /**
     * \brief function to return the current right motor torque
     *
     * This function returns the current right motor torque in Nm. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current right motor torque in Nm.
     *
     */
    float get_right_motor_torque(void);
    /**
     * \brief function to return the current operation mode
     *
     * This function returns the current operation mode of the segway platform. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current operation mode being used. The possible values are:
     *
     * - tractor
     * - balance
     * - poser down
     *
     */
    op_mode get_operation_mode(void);
    /**
     * \brief function to get the current gain schedule
     *
     * This function returns the current gain schedule of the segway platform. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the current gain schedule being used. The possible values are:
     *
     * - light
     * - tall
     * - heavy
     *
     */
    gain get_gain_schedule(void);
    /**
     * \brief function to return the value of the user battery voltage
     *
     * This function returns the current voltage of the user battery. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the value in volts of the user battery.
     */
    float get_ui_battery_voltage(void);
    /**
     * \brief function to get the current value of the powerbase battery voltage
     *
     * This function returns the current voltage of the powerbase battery. This 
     * function only returns the value of the internal attribute, but it does not
     * access the hardware platform. This value is periodically updated by the
     * feedback thread.
     *
     * \return the value in volts of the powerbase battery.
     *
     */
    float get_powerbase_battery_voltage(void);
    /**
     * \brief function to reset the segway platform
     *
     * This function is used to reset the segway platform to its default state. When
     * this function is called al configuration paramerets previoulsy set return to
     * its default value. Contrary to what happens with the motion commands, this 
     * function immediatelly sends a data packet to the segway platform.
     *
     * This function throws a CSegwayRMP200Exception exception to report any error.
     *
     */
    void reset(void);
    /**
     * \brief function to set new translational and rotational velocities
     *
     * This function is used to set a new motion command on the segway. This function 
     * sets the internal translational and rotational velocities to the desired values,
     * and it is the command thread which actually sends the new comamnd to the robot.
     *
     * This function can be called at any time to set up a new motion command to the 
     * robot. The command thread sends the current motion command to the robot every 
     * 20ms, so if this function is called more often, some of the commands will not
     * be executed. This function throws a CSegwayRMP200Exception to report errors.
     * 
     * \param vT desired translational velocity in meters per second. This parameter
     *           is limited to 12 kilometers per hour in both directions (both signs).
     *           However, the given value is affected by the velocity scale factor set
     *           by the set_velocity_scale_factor() function, so the actual maximum 
     *           speed can be lower.
     *
     * \param vR desired rotational velocity in revolutions per second. This parameter
     *           is limited to 0.6 revolutions per second in both directions (both
     *           signs). However, the given value is affectd by the turnrate scale 
     *           factor set by the set_turnrate_scale_factor() function, so the actual
     *           maximum turnrate can be lower.
     */
    void move(float vT,float vR);
    /**
     * \brief function to stop all motion
     *
     * This function is used to stop all motion on the segway. This function sets the
     * internal translational and rotational velocities to 0.0, and it is the command 
     * thread which actually sends the new comamnd to the robot.
     *
     * This function can be called at any time to stop the current motion of the robot.
     * This function throws a CSegwayRMP200Exception to report errors.
     *
     */
    void stop(void);
    /**
     * \brief function to close the segway driver
     *
     * This destructor activates the finish events to signal the feedback thread and 
     * the command thread to finish execution. Then it waits for the threads to end 
     * and destroy all internal data of the class.
     *
     */
    void close(void);
    // operators
    /**
     * \brief diplsay operator overloading
     *
     * This operator is used to show the current state of an object of this class
     * onto the standard ouput, file or any output stream. The information shown
     * is already formated as shown below:
     *
     * \verbatim
     * Pitch angle: <pitch_angle> degrees
     * Pitch rate: <pitch_rate> degrees/s
     * Roll angle: <roll_angle> degrees
     * Roll rate: <roll_rate> degrees/s
     * Left wheel velocity: <left_wheel_velocity> m/s
     * Right wheel velocity: <right_wheel_velocity> m/s
     * Yaw rate: <yaw_rate> degrees/s
     * Servo frames: <servo_frames> frames/s
     * Left wheel displacement: <left_wheel_displ> m
     * Right wheel displacement: <right_wheel_displ> m
     * Forward displacement: <forward_displ> m
     * Yaw displacement: <yaw_displ> rev
     * Left motor torque: <left_torque> Nm
     * Right motor torque: <right_torque> Nm
     * Operation mode: <op_mode>
     * Gain schedule: <gain_schedule>
     * UI battery voltage: <ui_battery> V
     * Powerbase battery voltage: <powerbase_battery> V
     * \endverbatim
     */
    friend std::ostream& operator<< (std::ostream& out, CSegwayRMP200& segway);
    /**
     * \brief destructor
     *
     * This destructor is called when the object is about to be destroyed. It calls
     * the close() function to safely free all the resources of the class. See the
     * documentation on this function for more details on the destruction process.
     *
     */
    ~CSegwayRMP200(); 
};

#endif

