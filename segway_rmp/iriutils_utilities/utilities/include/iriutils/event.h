// Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Author Sergi Hernandez  (shernand@iri.upc.edu)
// All rights reserved.
//
// This file is part of iriutils
// iriutils is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _EVENT_H
#define _EVENT_H

#include "mutex.h"
#include <string>

/**
 * \brief Implementation of a logical event
 *
 * This class implements the features of a logical event that can be signaled
 * and waited for from different threads or processes. This class has a CMutex 
 * object to avoid race conditions while signaling and waiting for the event. 
 * The event is created inactive by default, but it is possible to create it 
 * already signaled with additional parameters of the constructor.
 *
 * Each event has a unique identifier which is as string with a simple description
 * of the event that is used to identify the event. This identifier is passed by at
 * construction time and can not be modified afterwards. 
 *
 * The event is implemented as a pipe which can be written to (signal the event)
 * and read from (reset the event). The event is signaled by calling the set() 
 * function. If this function is called several times, all the instances are
 * stored in the pipe. The is_set() function can be used to check wether there
 * are instances of the event in the pipe (the event is active) or not (the 
 * event is not active). 
 *
 * Finally, the reset() function removes one of the event instances from the 
 * pipe. If there are several instances of the event stored, only one is removed
 * from the pipe, and when the last instance is removed from the pipe, the event
 * is no longer active. The function get_num_events() can be used to check how 
 * many events are pending in the internal pipe.
 *
 * The use of a pipe make it easy to implement wait functions because it is 
 * possible to use the select() function on the read file descriptor of the pipe.
 * Therefore, to peform a wait on a set of events, it is necessary to return the
 * read file descriptor of the pipe.
 *
 * Objects of this class can be used alone or they can be handled by an event
 * server implemented by the CEventServer class. See the documentation of this
 * class for a more detailed description. 
 *
 * This class uses exceptions to report errors. The name of the exception class
 * associated to this class is CEventException. See the corresponding 
 * documentation for a more detailes description.
 *
 */
class CEvent
{
  private:
    /**
     * \brief Read and write file descriptors of the internal pipe 
     *
     * This vector conatains the read and write file descriptors of the 
     * internal pipe. These values are automatically initialized when an
     * object of this class is created and can not be modified afterwards.
     *
     */
    int pipe_fd[2];

    /**
     * \brief Number of instances of the event 
     *
     * This value indicates how many instances of the corresponding event 
     * have ocurred so far. By default this value is initialized to $0$ if the
     * event is created inactive, and to $1$ if the event is active at 
     * construction time. This value is automatically updated when events are
     * set or reset.
     *
     */
    int num_activations;

    /** 
     * \brief A unique identifier of the current event
     *
     * This string has a unique identifier of the event that is used through 
     * out the code to take action on the desired event. This string is initialized 
     * at construction time and can not be modified afterwards.
     *
     */
    std::string event_id;

    /**
     * \brief A mutual exclusion object to properly handle the event
     *
     * This obejct avoids race conditions when setting and waiting for the 
     * event. When the event is set or reset, first the thread tries to lock
     * this mutex to avoid loosing event information, and releases it when 
     * it is finished. This object is automatically initialized when new object
     * is instantiated.
     *
     */
    CMutex access;
  protected:
    /**
     * \brief Function to set the event identifier
     *
     * This function sets the unique event identifier for each event. This 
     * function is protected, and therefore can only be executed inside the
     * class, because the identifier can not be modified after construction
     * of the event.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    void set_id(const std::string& id);

  public:
    /**
     * \brief Constructor with parameters
     * 
     * This constructor is used to create an initially inactive event with a 
     * given identifier. The values of the class attributes after this 
     * constructor is called are:
     *
     *     - pipe_fd[0] = read file descriptor of the pipe.
     *     - pipe_fd[1] = write file descriptor of the pipe.
     *     - num_events = 0.
     *     - event_id = event identifier.
     *     - access = a new mutual exclusion object.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     * \param id: a null terminated string with the event identifier. The
     * identifier is stored internally in a different memory location so that 
     * the parameter can be freed after calling this constructor. The identifier 
     * must not have any spaces or special characteres.
     *
     */
    CEvent(const std::string& id);

    /** 
     * \brief Constructor with parameter
     *
     * This constructor is used to crate a new event which can be active by 
     * default with a given identifier. The behavior of this constructor is 
     * equivalent to the first constructor except that depending on the value of 
     * init_state, the event is active or not by default.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     * \param id: a null terminated string with the event identifier. The 
     * identifier is stored internally in a different memory location so that
     * the parameter can be freed after calling this constructor. The identifier
     * must not have any spaces or special characters.
     *
     * \param create_signaled: a boolean that indicates if the event must be active 
     * by default ($1$) or not ($1$).
     */
    CEvent(const std::string& id,bool create_signaled);

    /**
     * \brief Function to reset the event.
     *
     * This function reads a byte from the internal pipe to reset the event. If
     * there is only one instance of the event in the internal pipe the event 
     * become inactive, but if there are several instances of the event, its state
     * does not change, and only the number of pending event is decreased.
     *
     * Internally, the mutex is locked and released to avoid race conditions when
     * several threads are trying to activate or reset the event. 
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     */
    void reset(void);

    /** 
     * \brief Function to activate the event.
     *
     * This function writes a byte to the internal pipe to activate the event. 
     * If the event is currently inactive, it is activated, and if it is already 
     * active its state does not change, except for a new instance of the event 
     * in the pipe.
     *
     * Internally, the mutex is locked and released to avoid race conditions when
     * several threads are trying to activate or reset the event. Each time this 
     * function is called, the number of event instances is incremented by $1$.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    void set(void);

    /** 
     * \brief Function to wait for the activation of the event
     * 
     * This function waits for the activation of the event for a given period of 
     * time. If the event is not active after the desired time elapses, the 
     * function returns anyway, throwing an exception. 
     *
     * If no timeout is desired, any negative value will be interpreted as an 
     * infinite wait time. Passing a negative parameter is possible but not 
     * recomnded because it may block a thread forever.
     *
     * When the event is activated, all the threads blocked in a wait() function
     * for the activated event will awake, and the evetn will be automatically 
     * reseted.
     *
     * \param timeout_us Tis is an integer which indicates the desired timeout
     * value in micro-seconds. This value may be negative if no timeout is 
     * desired.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     * 
     */
    void wait(int timeout_us);

    /** 
     * \brief Function to check the state of the event.
     *
     * This functions checks wether the event is active ($1$) or not ($0$), but
     * it does not take into account if there are several instances of the event 
     * in the internal pipe. This function does not change the internal state of 
     * the event.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     * \return A boolean that indicates if the event is set (true) or not 
     * (false).
     */
    bool is_set(void);

    /**
     * \brief Function to get the number of event instances.
     *
     * This function returns the number of instances of the event currently in 
     * the internal pipe. This function does not change the number of instances
     * of the event in the pipe.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     * \return An integer with the number of insatnces of the event in the pipe-
     * There is no limit in the number of events that can be stored in the pipe.
     */
    int get_num_activations(void);

    /**
     * \brief Function to get the event identifier
     *
     * This fucntion returns the event identifier of the event as a null 
     * terminated string. 
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     * \return A reference to a non-allocated memory space where the 
     * identifier of the event is to be copied. The necessary memory is allocated
     * internally to match the size of the identifier. The calling process is 
     * responsible of freeing the allocated memory.
     *
     */
    std::string get_id(void);

    /**
     * \brief Function to get the read file descriptor of the pipe
     *
     * This function returns the read file descriptor of the internal pipe. This
     * file descriptor must not be used anywhere except to wait for several 
     * events simultaneously. Closing the returned value will cause unexpected 
     * errors.
     *
     * \return An integer with the read file descriptor of the internal pipe. 
     */
    int get_fd(void);

    /** 
     * \brief Destructor
     * 
     * This destructor safely frees all the allocated memory and destroys the 
     * mutual exclusion method.
     *
     * This function throws a CEventException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    virtual ~CEvent();
};

#endif
