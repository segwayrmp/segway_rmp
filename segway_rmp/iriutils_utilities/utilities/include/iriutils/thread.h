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

#ifndef _THREAD_H
#define _THREAD_H

#include "mutex.h"
#include <string>

#include <pthread.h>

typedef enum{attached,starting,active,detached} thread_states;

/**
 * \brief Implementation of a parallel thread of execution
 *
 * This class handles all thread related issues to hide them from the final
 * user. This class can only handle a single thread of execution. This class
 * already has a CMutex object to avoid that the data shared by several
 * threads gets corrupted due to simultaneous accesses.
 *
 * Similar to events, threads also have a unique identifier in form of a 
 * string that is assigned at construction time and that can not be modified
 * afterwards. This identifier is used through out the code to identify the 
 * thread on which to execute the desired action.
 *
 * The thread itself is created at construction time, but it is useless at 
 * this point. First, it is necessary to assign the function that will 
 * execute the thread with the attach() function. After that, calling the 
 * start() function, the thread will begin its execution in parallel with all 
 * other threads. 
 *
 * The thread can finish its execution by its own or else, it can be endend by
 * the user calling the end function. If the thread gets stuck at some point 
 * and does not end when it is requested, the user can call the kill() 
 * function to immediatelly terminate the thread. Hoever, in this second case,
 * all dynamically allocated memory won't be freed.
 *
 * When the thread is no longer in execution, it is possible to remove the 
 * assigned function by calling the detach() method and assign a new function 
 * by calling again the attach() function. 
 *
 * At any point it is possible to retrieve the current state of the thread 
 * using the get_thread_state() function. The possible states of the thread 
 * are:
 *
 *  - attached: when a function is assigned but the thread is not running.
 *  - starting: when the thread is about to start execution.
 *  - active: when the thread is in execution.
 *  - detached: when the thread is created but has no ssigned function. This 
 *  is the default state.
 *
 *  The different states and the transition conditions are shown in the next 
 *  figure:
 *
 *  \image html thread_states.png
 *  \image latex thread_states.eps "Thread states" width=10cm
 *
 * Also the thread identifier can be retrieved using the get_thread_is() function. 
 */
class CThread
{
  private:
    /**
     * \brief Thread information structure
     *
     * This structure hold system level information of the thread and it is 
     * initialized when the thread is first created. This information can not 
     * be modified at nay time and it is not accessible from outside the class.
     *
     */
    pthread_t thread;

    /** 
     * \brief User thread function
     *
     * This attribute is a pointer to the user function that will be executed 
     * as the concurrent thread. The function prototype must be as follows:
     *
     * \verbatim
     *   void *<function_name>(void *param)
     * \endverbatim
     *
     * If the function belongs to a class, it must be defined as static to 
     * avoid the implicit pointer to the corresponding object taht is passed 
     * always as the first parameter of all class functions. If the function 
     * needs access to the class attributes, the this pointer can be passed 
     * as an explicit paramter to the function and then casted to the correct 
     * class.
     *
     * The parameter to the function is assigned at creation time and it is a
     * pointer to the object where the user thread function belongs.
     *
     */
    void *(*user_thread_function)(void *param);

    /**
     * \brief Thread function parameter
     *
     * This pointer is a reference to the argument that is to be passed to the
     * thread function. This variable can take any value, even NULL. If the
     * thread function is a member of another class, normally this parameter
     * will be a reference to the object itslef, so that the function can have
     * access to the class attributes.
     *
     * By default this parameter is set to NULL, and can only be changed when 
     * assigning the thread function with the attach() function. Its value is 
     * automatically updated whenever a new function is set.
     *
     */
    void *param;

    /** 
     * \brief Current state of the thread
     *
     * This variable stores the current state of the thread. By default it is 
     * initialized to detached, and its value is automatically updated when 
     * the thread changes its states. The possible states of the thread are:
     *
     *   - attached: when the thread has a function assigned but it is not 
     *   executing.
     *   - starting: when the threda is about to start execution.
     *   - active: when the thread is executing.
     *   - detached: when the thread is created but has no assigned function. 
     *   This is the default state.
     *
     */
    thread_states state;

    /**
     * \brief A unique identifier of the current thread
     *
     * This string has a unique identifier of the thread that is used through     
     * out the code to take action on the desired thread. This string is 
     * initialized at construction time and can not be modified afterwards.
     */
    std::string thread_id;

    /**
     * \brief A mutual exclusion object to properly handle the thread.
     *
     * This object avoids race conditions when starting and cancelling 
     * threads. When any action is required on the thread information, the 
     * thread first lock this mutex to avoid information corruption, and 
     * releases it when it is done. The object is automatically initialized 
     * when a new thread is created.
     */
    CMutex access;

  protected:
    /**
     * \brief Internal thread function
     * 
     * This function is the main thread function called when the thread is 
     * started. This function changes the current state of the thread to 
     * active and calls the user defined function. If no user function is 
     * assigned, the function throws an exception.
     *
     * This function also handles the cleanup functions necessary to free all 
     * dynamically allocated resources when the thread is cancelled.
     *
     * \param param This parameter is assigned when the thread is first 
     * created and it is a pointer to the object where the user thread 
     * function belongs.
     */
    static void *thread_function(void *param);

    /**
     * \brief Function to set the thread identifier
     *
     * This function sets the unique identifier for each thread. This
     * function is protected, and therefore can only be executed inside the
     * class, because teh identifier can not be modified after construction
     * of the thread.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    void set_id(const std::string& id);

  public:
    /**
     * \brief Constructor
     *
     * This constuctor creeates the thread itself, initializes the thread 
     * information structure and assigns it a unique identifier. However, 
     * after creation the thread is still unusable because there is no 
     * assigned function. The values of the class attributes after this 
     * constructor is called are:
     *    
     *    - thread_id = -1
     *    - thread = (uninitialized)
     *    - user_thread_function = NULL;
     *    - thread_id = thread_id
     *    - access = a new mutual exclusion object
     *
     * \param id: a null terminated string with the thread identifier. 
     * The identifier is stored internally in a different memory location so 
     * that the parameter can be freed after calling this constructor. The
     * identifier must not have any spaces or special characters.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     */
    CThread(const std::string& id);

    /**
     * \brief Function to attach the user function of the thread
     *
     * This function assigns a user defined function to the thread. If there 
     * is already an assigned function, the older one is lost. It is 
     * impossible to change the thread function while the thread is in 
     * execution.
     *
     * The user function will normally be a member function of another class, 
     * and it must be defined as static to remove the implicit 'this' parameter.
     * So, in order to have access to the class, the parameter of the function
     * must be the this pointer. In this case there is no direct access from the
     * thread function, to the thread attributes.
     *
     * Care must be taken when allocating dynamic memory inside the thread
     * function because when the thread is cancelled, no memory is freed. This
     * function can be called whenever the thread is not in execution to change 
     * the desired execution function, but it has no effect when the thread is
     * started. When the function is changed, the previous one is lost and can 
     * not be recovered.
     *
     * This function changes the state of the thread from detached to attached.
     *
     * \param user_thread_function A reference to the function that will execute
     * the thread when started. The declaration of the function must be as 
     * follows:
     *
     *     \verbatim
     *     void *<function_name>(void *)
     *     \endverbatim
     *
     *
     * \param param A reference to the paramater that needs to be passed as an 
     * argument to the thread function. If the thread function is a member of a
     * class, it must be a pointer to the object where the function belong in 
     * order to have access to the class attributes. This parameter can be of
     * any type, even NULL.
     * 
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    void attach(void *(*user_thread_function)(void *param),void *param);

    /**
     * \brief Function to start the execution of the thread
     *
     * This function is used to start the execution of a thread. This function 
     * must be called after attaching a function to the thread, or otherwise it 
     * will throw an exception. If this function is called when the thread is 
     * already active, nothing happens.
     *
     * This function starts the internal thread function to set the appropiate 
     * thread state, and then it calls the user thread function. This function 
     * changes the state of the thread from attached to starting. When the actual
     * thread function is changed, the state changes to active.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     * 
     */
    void start(void);

    /**
     * \brief Function to request the end of the execution of the thread
     *
     * This function sends a request to the thread in order to terminate it. 
     * If the thread is not active, this function has no effect. Otherwise, 
     * this function won0t return until the thread has finished.
     *
     * This function only ends the execution of the thread, but it does not
     * delete nor change the user thread function. After calling this function,
     * it is possible to restart the thread with the same function calling the
     * start() function again.
     *
     * This function changes the state of the thread from active to attached.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    void end(void);

    /** 
     * \brief Function to immediatelly terminate the execution of the thread
     *
     * This function is similar to the end() function, but in this case the
     * thread is killed whatever its state, instead of requesting it to end.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    void kill(void);

    /**
     * \brief Function to remove the assigned user function
     *
     * This function is used to remove the user function to be executed in 
     * the thread. It is not necessary to call this function to change the
     * desired execution function, it is possible to call the attach()
     * function to change the user function.
     *
     * If there is no function attached to the thread, this function does 
     * nothing, and if there is one, the reference is set to NULL to delete
     * the reference to the previous thread function. If the thread is active,
     * nothing is done because the thread needs to be stoped before by calling
     * the end() function.
     *
     * This function changes the state of the thread from attached to detached.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    void detach(void);

    /**
     * \brief Function to retrieve the state of the thread
     *
     * This function returns the current state of the thread. After creation, 
     * the thread is in the detached state. After assigning a user function with 
     * the attach() function, it changes to the attached state. When the thread
     * is started, it first changes to the starting state, which is changed when 
     * the actual user thread function is finally executed. 
     * 
     * Finally, when the thread is stopped or killed, the state is changed to 
     * detached again. This funciton can be used to know the current state of the
     * thread before performing any action.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     * \return This function returns an integer with the current state of the 
     * thread. The possible values of this values are:
     *    - attached.
     *    - starting.
     *    - active.
     *    - detached.
     *
     */
    int get_state(void);

    /**
     * \brief Function to get the thread identifier
     *
     * This function returns the thread identifier of the thread as a null 
     * terminated string.
     *
     * This function throws a CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     *
     * \return A reference to a non-allocated memeory space where the
     * identifier of the thread is to be copied. The necessary memory is allocated
     * internally to match the size of the identifier. The calling process is
     * responible of freeing the allocated memory.
     *
     */
    std::string get_id(void);

    /**
     * \brief Destructor
     *
     * This destructor safely finishes the thread. If it is active, it is 
     * automatically killed and all the allocated memory is freed and the mutex is 
     * destroyed.
     *
     * This function throwsa CThreadException if there is an error. See the
     * corresponding documentation for a more detailed description of these
     * exceptions.
     */
    virtual ~CThread();
};

#endif
