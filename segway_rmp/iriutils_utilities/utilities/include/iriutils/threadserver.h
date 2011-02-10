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

#ifndef _THREAD_SERVER
#define _THREAD_SERVER

#include <list>
#include "thread.h"
#include "mutex.h"

/** 
 * \brief Global thread server
 *
 * This class implements a thread server which is global to the application
 * and also only one instance exist that is shared by all objects requiring
 * threads.
 *
 * There is no limit in the number of threads that can be created in the whole 
 * application. Each of this threads is assigned a unique identifier at creation
 * time which is provided by the user. All identifier must be different, an 
 * they are used afterwards to access the desired thread.
 *
 * By using an identifier to access the desired thread, this class also isolates
 * the low level details of the threads implemented by the CThread class by
 * providing a public interface that allows access to all thread capabilities.
 * Doing so, there is no direct access to the object outside the server.
 *
 * The public interface allows to start and stop threads, attach and dettach
 * thread functions to them and also check its current state in real time.
 * Internally, it has a complete list of all threads created in the application
 * so that any object or thread can access any thread if its identifier is known.
 *
 * This class also has a CMutex object to avoid race conditions or data
 * corruption while several threads of execution are trying to access the same
 * thread. 
 *
 * This class uses exceptions to report errors. The name of the exception 
 * class associated to this class is CThreadServerException. For a more detailes
 * descritpion, see the corresponding documentation.
 *
 */
class CThreadServer
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
    static CThreadServer* pinstance;
    /**
     * \brief Thread handler
     *
     * This static list of threads can be used by all objects in an application
     * and it is used to handle all the threads in an unified way. All threads
     * are created and handled through the public interface of the class and 
     * can be referenced by its identifier. By default this list is empty.
     */
    std::list<CThread> thread_list;
    /// Mutual exclusion object for the thread handler
    /** This mutual exclusion object is used to handle several threads accessing
     * the list of acvtive threads. Several CMutex objects are used to avoid 
     * blocking the events while handling the threads.
     */
    CMutex access_threads;
  protected:
    /**
     * \brief Default constructor
     *
     * This constructor initializes the thread list and the mutex object to access
     * it. The list can only be modified through the public interface of the 
     * class.
     *
     * The reference to the newly created object is not modified. This constructor 
     * is only called once and from inside the instance() function. It can not
     * be called directly by the user since it is declared as protected.
     */ 
    CThreadServer();
    /** 
     * \brief Copy constructor
     *
     * This constructor is used to initialize a new object with the contents of
     * an existing one. Since there could be only one instance of this class,
     * only the pinstance attribute must be copied, but since it is static
     * nothing is to be done in this constructor.
     *
     * \param object an existing instance of a CThreadServer class which has been
     *               already initialized.
     */ 
    CThreadServer(const CThreadServer& object);
    /**
     * \brief assign operator overloading
     *
     * This function overloads the assign operator for this class. Since there 
     * could be only one instance of this class, only the pinstance attribute 
     * must be copied, but since it is static, nothing is to be done.
     *
     * \param object an existing instance of a CThreadServer class which has been
     *               already initialized.
     *
     */ 
    CThreadServer& operator = (const CThreadServer& object);
    /**
     * \brief Function to search for an specific thread
     *
     * This function is used to search for a particular thread inside the
     * thread handler list. It is protected because no one outside the class 
     * can have access to the particular CThread objects. 
     *
     * If the required thread exists, a reference to it is returned by the 
     * function, but if there is no thread with the provided identifier, the 
     * function return a NULL reference, but does not throw any exception.
     *
     * 3YThis function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id A null terminated string that identifies the desired
     * thread. The necessary memory for this string must be allocated before 
     * calling this function.
     *
     * \return A reference to the desired CThread object, or a NULL reference
     * if the thread with the provided identifier does not belong to the 
     * thread handler.
     */
    std::list<CThread>::iterator search_thread(const std::string& thread_id);
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
    static CThreadServer* instance(void);
    // Thread handler functions
    /**
     * \brief Function to create a new thread
     *
     * This function creates a new thread with a given identifier and adds it to 
     * the thread handler. If there is a thread in the internal list with the same
     * identifier, the new thread is not added and an exception is thrown. 
     * Otherwise, the new thread is added to the list of threads and can be accessed
     * immediatelly by other threads using the provided identifier.
     *
     * This function locks the access_threads Mutex object to avoid other threads 
     * concurrently adding new threads which could cause unpredictible errors. After
     * calling this function, the only way to access the newly cretaed thread is 
     * through its identifier, the user does not have any reference to the CThread
     * object inserted into the thread list.
     *
     * This function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id a null terminated string with the thread identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     *
     */
    void create_thread(const std::string& thread_id);

    /**
     * \brief Function to delete an existing thread
     *
     * This function removes an existing thread from the thread handler. If the
     * desired thread is not in the internal list, an exception is thrown and 
     * nothing happens. Otherwise, the thread is removed, and future reference
     * to it will end with an exception thrown.
     *
     * This function locks the access_threads Mutex object to avoid other threads
     * concurrently removing threads which could cause unpredictable errors. This
     * function throws an exception if there is any other error.
     *
     * This function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id a null terminated string with the thread identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     */
    void delete_thread(const std::string& thread_id);

    /**
     * \brief Function to attach a function to a thread
     * 
     * This function provides access to the private CThread objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the attach() function of the CThread 
     * class.
     *
     * The other parameters to the function are descrived in the CThread class
     * reference.
     *
     * This function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id a null terminated string with the thread identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     */
    void attach_thread(const std::string& thread_id,void *(*user_thread_function)(void *param),void *param);

    /** 
     * \brief Function to start a thread
     *
     * This function provides access to the private CThread objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the start() function of the CThread
     * class.
     *
     * This function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id a null terminated string with the thread identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     */
    void start_thread(const std::string& thread_id);

    /**
     * \brief Function to request the termination of a thread
     *
     * This function provides access to the private CThread objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the end() function of the CThread
     * class.
     *
     * This function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id a null terminated string with the thread identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     */
    void end_thread(const std::string& thread_id);

    /** 
     * \brief Function to immediately terminate a thread
     *
     * This function provides access to the private CThread objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the kill() function of the CThread
     * class.
     *
     * This function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id a null terminated string with the thread identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     *
     */
    void kill_thread(const std::string& thread_id);

    /**
     * \brief Function to detach a function from a thread
     *
     * This function provides access to the private CThread objects through the 
     * public interface of the class. To see a more detailed description of its
     * features, see the documentation on the detach() function of the CThread
     * class.
     *
     * This function throws a CThreadServerException in case of any error. See
     * the corresponding documentation for a more detailed decription of this
     * exception class.
     *
     * \param thread_id a null terminated string with the thread identifier. The 
     * identifier is stored internally in a different memeory location so that
     * the parameter can be freed after calling this function. The identifier 
     * must not have any spaces or special characters, and must be unique.
     */
    void detach_thread(const std::string& thread_id);
};

#endif
