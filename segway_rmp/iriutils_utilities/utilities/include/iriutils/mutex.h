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

#ifndef _MUTEX_H
#define _MUTEX_H

#include <pthread.h>

/**
 * \brief Implementation of a mutual exclusion mechanism
 *
 * This class implements the mutual exclusion mechanism. Any class that needs 
 * to handle shared resources must have an object of this class as a member
 * to avoid that multiple threads of execution simultaneoulsy modify memory or
 * peripherals in a critical section.
 *
 * Any thread that needs to access shared resources must first try to lock the 
 * mutual exclusion mechanism by calling the enter() function. If the critical 
 * section is free, the current thread will be granted access but any other 
 * thread will get blocked at the enter() function call. Otherwise, if the 
 * shared memory is already in use, the curretn thread will be blocked until 
 * the mutex is released by the current owner by calling the exit() function.
 *
 * The necessary information of the mutex is initialized when an object of this
 * class is createdd, and automatically destroyed when the object is deleted. 
 * This class uses exceptions to report error, and the name of the associated
 * exception class is CMutexException. For a more detailed description of
 * this class, see the corresponding documentation.
 *
 */
class CMutex
{
  private:
    /**
     * \brief  variable to handle the mutual exclusion mechanism
     *
     * this variable is automatically initialized when an object of this class
     * or any inherited class is created and holds all the necessary information 
     * to handle any shared resource.
     *
     */
    pthread_mutex_t access;

  public:
    /** 
     * \brief function to request access to the critical section
     * 
     * this function must be call before accessing any shared resource to 
     * block access to any other thread. If the resource is free the function 
     * will return immediatelly, and it will get blocked if there is another 
     * thread accessing the shared resources.
     *
     * This function throws a CMutexException in case of any error. See the
     * corresponding documentation for a more detailed description.
     *
     */
    void enter(void);

    /** 
     * \brief function to request access to the critical section and return
     * if denied.
     * 
     * If you'd like to access a resource when it is available, but continue 
     * if the resource is not available, then call this function before 
     * accessing the shared resource. If the resource is free it will return
     * true immediatelly, and false if there is another thread accessing the
     * shared resources.
     *
     * If successfull, this function also locks the mutex, so it is necessary
     * to release the mutex if the return value is true. Otherwise, the mutex
     * will block the next time it is accessed.
     *
     * This function throws a CMutexException in case of any error. See the
     * corresponding documentation for a more detailed description.
     *
     * \return true if the lock was adquired, false otherwise
     */
    bool try_enter(void);

    /**
     * \brief function to release the critical section
     *
     * this function must be called after accessing the shared resources to
     * free the critical section an allow other threads to access it. If this
     * function is not called, the shared resource will be blocked forever and 
     * so will be all the threads trying to access it.
     *
     * If this function is called more than once without calling the enter()
     * function, it will not fail, but the mutex will behave erratically and
     * it may fail.
     *
     * This function throws a CMutexException in case of any error. See the
     * corresponding documentation for a more detailed description.
     *
     */
    void exit(void);

    /**
     * \brief default constructor
     *
     * this constructor creates the mutex and initializes it to be used 
     * throughout the code. If an error ocurrs, an exception is thown that must
     * be handled by a try ... catch environment.
     *
     * This function throws a CMutexException in case of any error. See the
     * corresponding documentation for a more detailed description.
     *
     */
    CMutex();

    /**
     * \brief default destructor
     *
     * the destructor tries to destroy the mutex and throws and exception in 
     * case of any error. The excpetion must be handled by a try ... catch 
     * environment.
     *
     * This function throws a CMutexException in case of any error. See the
     * corresponding documentation for a more detailed description.
     */
    virtual ~CMutex();
};

#endif
