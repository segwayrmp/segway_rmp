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

#ifndef _EVENT_SERVER
#define _EVENT_SERVER

#include <list>
#include "event.h"
#include "mutex.h"

/**
 * \brief Global event server
 *
 * This class implements an event server which is global to the application
 * and also only one instance exist that is shared by all objects requiring
 * events.
 *
 * There is no limit in the number of events that can be created in the whole 
 * application. Each of this events is assigned a unique identifier at creation
 * time which is provided by the user. All identifier must be different, an 
 * they are used afterwards to access the desired event.
 *
 * By using an identifier to access the desired event, this class also isolates
 * the low level details of the events implemented by the CEvent class by
 * providing a public interface that allows access to alll event capabilities.
 * Doing so, there is no direct access to the object outside the server.
 *
 * The public interface allows to create and destroy events, signal and reset
 * them, check its current state and also wait for a given set of them. 
 * Internally, it has a complete list of all event cretaed in the application
 * so that any object or thread can use any event if its identifier is known.
 *
 * This class also has a CMutex object to avoid race conditions or data
 * corruption while several threads of execution are handling the same event
 * in the server. 
 *
 * This class uses exceptions to report errors. The name of the exception 
 * class associated to this class is CEventServerException. For a more detailes
 * descritpion, see the corresponding documentation.
 *
 */
class CEventServer
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
    static CEventServer* pinstance;
    /**
     * \brief Event handler
     *
     * This static list of events can be used by all objects in an application
     * and it is used to handle all the events in an unified way. All events
     * are created and handled through the public interface of the class and 
     * can be referenced by its identifier. By default this list is empty.
     */
    std::list<CEvent> event_list;
    /// Mutual exclusion object for the thread handler
    /** This mutual exclusion object is used to handle several threads accessing
     * the list of acvtive threads. Several CMutex objects are used to avoid 
     * blocking the events while handling the threads.
     */
    CMutex access_events;
  protected:
    /**
     * \brief Default constructor
     *
     * This constructor initializes the event list and the mutex object to access
     * it. The list can only be modified through the public interface of the 
     * class.
     *
     * The reference to the newly created object is not modified. This constructor 
     * is only called once and from inside the instance() function. It can not
     * be called directly by the user since it is declared as protected.
     *
     */ 
    CEventServer();
    /** 
     * \brief Copy constructor
     *
     * This constructor is used to initialize a new object with the contents of
     * an existing one. Since there could be only one instance of this class,
     * only the pinstance attribute must be copied, but since it is static
     * nothing is to be done in this constructor.
     *
     * \param object an existing instance of a CEventServer class which has been
     *               already initialized.
     *
     */ 
    CEventServer(const CEventServer& object);
    /**
     * \brief assign operator overloading
     *
     * This function overloads the assign operator for this class. Since there 
     * could be only one instance of this class, only the pinstance attribute 
     * must be copied, but since it is static, nothing is to be done.
     *
     * \param object an existing instance of a CEventServer class which has been
     *               already initialized.
     */ 
    CEventServer& operator = (const CEventServer& object);
    /** 
     * \brief Function to search for an specific event
     *
     * This function is used to search for a particular event inside the
     * event handler list. It is protected because no one outside the class 
     * can have access to the particular CEvent objects. 
     *
     * If the required event exists, a reference to it is returned by the 
     * function, but if there is no event with the provided identifier, the 
     * function return a NULL reference, but does not throw any exception.
     *
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param event_id A null terminated string that identifies the desired
     * event. The necessary memory for this string must be allocated before 
     * calling this function.
     *
     * \return A reference to the desired CEvent object, or a NULL reference
     * if the event with the provided identifier does not belong to the 
     * event handler.
     */
    std::list<CEvent>::iterator search_event(const std::string& event_id);
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
    static CEventServer* instance(void);
    // Event handler functions
    /**
     * \brief Function to create a new event
     *
     * This function creates a new event with a given identifier and adds it to 
     * the event handler. If there is an event in the internal list with the same
     * identifier, the new event is not added and an exception is thrown. 
     * Otherwise, the new event is added to the list of events and can be accessed
     * immediatelly by other threads using the provided identifier.
     *
     * This function locks the access_events Mutex object to avoid other threads 
     * concurrently adding new events which could cause unpredictible errors. After
     * calling this function, the only way to access the newly cretaed event is 
     * through its identifier, the user does not have any reference to the CEvent
     * object inserted into the event list.
     *
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param event_id a null terminated string with the event identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     */
    void create_event(const std::string& event_id);
    /**
     * \brief Function to delete an existing event
     * 
     * This function removes an existing event from the event handler. If the
     * desired event is not in the internal list, an exception is thrown and 
     * nothing happens. Otherwise, the event is removed, and future reference
     * to it will end with an exception thrown.
     *
     * This function locks the access_events Mutex object to avoid other threads
     * concurrently removing events which could cause unpredictable errors. This
     * function throws an exception if there is any other error.
     *
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     */
    void delete_event(const std::string& event_id);
    /**
     * \brief  Function to activate an event
     *
     * This function provides access to the private CEvent objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the set() function of the CEvent
     * class.
     * 
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param event_id: a null terminated string with the event identifier. The
     * identifier is stored internally in a different memory location so that 
     * the parameter can be freed after calling this constructor. The identifier 
     * must not have any spaces or special characteres.
     */
    void set_event(const std::string& event_id);
    /**
     * \brief Function to deactivate an event
     *
     * This function provides access to the private CEvent objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the reset() function of the CEvent
     * class.
     *
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param event_id: a null terminated string with the event identifier. The
     * identifier is stored internally in a different memory location so that 
     * the parameter can be freed after calling this constructor. The identifier 
     * must not have any spaces or special characteres.
     */
    void reset_event(const std::string& event_id);
    /**
     * \brief Function to query if the event is active or not
     *
     * This function provides access to the private CEvent objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the is_set() function of the CEvent
     * class.
     *
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param event_id: a null terminated string with the event identifier. The
     * identifier is stored internally in a different memory location so that 
     * the parameter can be freed after calling this constructor. The identifier 
     * must not have any spaces or special characteres.
     */
    bool event_is_set(const std::string& event_id);
    /** 
     * \brief function to get the number of event
     *
     * This functions returns the number of events currently in the internal
     * event handler. This function locks the access_events Mutex object to avoid
     * new events to be added or existing ones to be removed while accssing the
     * number of elements in the internal list. This function throws an exception 
     * if there is any error.
     *
     * \return An integer with the number of event sin the event handler. This 
     * value is always non-negative, but ther is no upper limit on the number
     * of concurrent events.
     */
    int get_num_events(void);
    /** 
     * \brief Function to get the number of activations of a single event
     *
     * This function provides access to the private CEvent objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the get_num_activations() function of 
     * the CEvent class.
     *
     * \param event_id: a null terminated string with the event identifier. The
     * identifier is stored internally in a different memory location so that 
     * the parameter can be freed after calling this constructor. The identifier 
     * must not have any spaces or special characteres.
     */
    int get_num_activations(const std::string& event_id);
    /** 
     * \brief Function to wait for the first event in a set
     *
     * This function waits on a set of events for the first one to became 
     * active. The events to be waited upon are referenced by its unique 
     * identifier and passed to the function through a list. 
     *
     * If all the event identifiers exist in the internal event handler, all
     * their file descriptors are retieved and waited upon simultaneoulsy. If 
     * one or more of the identifiers do not exist, an exception is thrown.
     *
     * If all the desired events exist, the function gets blocked until the 
     * first event is activated. Then, the active event is reset automatically
     * and the position of the event in the event list is returned to inform
     * the calling process which of the event has been activated.
     *
     * If non of the event get active, the function is blocked forever. This
     * function locks the access_events Mutex obejct except when it is actually
     * waiting for the events, to allow other threads to activate the necessary 
     * events. Otherwise all the threads handling events will get blocked 
     * forever. This function throws an exception if there is any error.
     *
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param events an STL list of constant references to event identifiers. 
     * This list must be created before calling this function, and several lists
     * can be used with different sets of events in a single application. The 
     * number of events in this list is only limited by the number of events
     * in the event handler. If one identifier appears sevaral times, only the 
     * first one is taken into account.
     *
     * \return an integer with the position of the active event in the list of
     * events to wait upon passed as an argument to the function. This value is 
     * always non-negative and its maximum value is the number of events - 1.
     *
     * \param timeout an integer value with the maximum ammount of time to wait 
     * for an event in miliseconds. If an event ocurrs before the specified time
     * nothing happens, otherwise an exception is thrown. By default its value is
     * -1 which indicates no timeout.
     *
     */
    int wait_first(std::list<std::string> events,int timeout = -1);
    /**
     * \brief Function to wait for all the events is a set
     *
     * This function is very similar to the wait_first() function, except that
     * this one waits for all events in the list to became active. In this case
     * there is no need to return any index value because the function will only 
     * finish when all the events are active.
     *
     * The events that have been activated are actually removed from the list of 
     * event to wait upon, so that only the first activation of an event is 
     * actually handled by a call to this function. All the activated events are
     * internally reset.
     *
     * This function throws a CEventServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param events an STL list of constant references to event identifiers. 
     * This list must be created before calling this function, and several lists
     * can be used with different sets of events in a single application. The 
     * number of events in this list is only limited by the number of events
     * in the event handler. If one identifier appears sevaral times, only the 
     * first one is taken into account.
     *
     * \param timeout an integer value with the maximum ammount of time to wait 
     * for an event in miliseconds. If an event ocurrs before the specified time
     * nothing happens, otherwise an exception is thrown. By default its value is
     * -1 which indicates no timeout.
     */
    void wait_all(std::list<std::string> events, int timeout = -1);
};

#endif
