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

#ifndef _LOG_H
#define _LOG_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <vector>
#include "mutex.h"
#include "ctime.h"

/** 
 * \brief Implementation of a log file
 *
 * This class allows easyliy handling log files in a class. The name of the 
 * file in which to save the user messages is fixed at construction time, and
 * the corresponding file created. If a file with the same name already exists,
 * it is deleted and a new one is created. The file is opened at construction
 * time, and remains this way until the class is destroyed, even if the log is
 * disabled.
 *
 * The message logging for a class can be enabled or disabled at any time. When 
 * enabled, the user can use the log() function to save messages into the file.
 * These messages will each have the time stamp in which they where written. When
 * disabled, the log() fnction will do nothing.
 *
 * The function is_enabled() make it possible to check if the log is enabled or 
 * not. By default, the ./log/ directory is used to store all the log files. The 
 * class provides a mutex object in order to avoid data corruption when multiple 
 * threads try to log messages to this object.
 */
class CLog
{
  private:
    /** 
     * \brief Log file structure
     *
     * This structure hold all the information regarding the file. It is initialized
     * when the object is created and associated to the user provided filename. It
     * is not possible to modify this structure directly, it will by updated when the
     * log() function is used, and destroyed when the corresponding object is 
     * destroyed.
     */
    std::fstream log_file;

    /**
     * \brief Log filename  
     *
     * This string contains the full name of the log file including the path and 
     * default extension. It is initialized at construction time with the name
     * provided by the user and can not be modified afterwards. It is possible
     * to get the full filename using the get_filename() function.
     */
    std::string filename;
    /**
     * \brief Enable flag
     *
     * Thif flag indicates if the particular log is enabled (true) or not (false). 
     * Its default value is true, so that when a new object of this class is created,
     * it can immediatelly start logging user messages. 
     */
    bool enabled;

    /**
     * \brief A mutual exclusion object to properly handle the log file
     *
     * This object avoid that the messages written to the log file get corrupted due
     * to concurrent write operations to a single file from different threads. When a 
     * thread wants to log a message, it first locks the mutex, then writes the message 
     * and finally frees the mutex.
     */
    CMutex access;
    
    /**
     * \brief A time object to create the time stamps
     *
     * This attribute is used to get the current computer time each time a new log entry 
     * is created, and also output it in the desired format to be written into the log
     * file. This object is initialized each time the log() function is called and can 
     * not be directlyy accessed by the user.
     */
    CTime time_stamp; 
  public:
    /** 
     * \brief Constructor
     *
     * This function initializes the file structure and associates it with the filename
     * provided by the user. By default, both global and local enables are set to true.
     * The values of all the attributes after calling this constructor is as follows:
     *
     *     - log_file: An initialized FILE structure.
     *     - enable=true.
     *
     * \param filename a null-terminated string with the name of the log file to be
     * created. The memory necessary for this string must be allocated by the calling
     * process. The file name must not have any spaces or special character. The 
     * filename must not have any extension, '.log' is automatically appended.
     *
     * This function throws a CLogException if there is any error.
     */
    CLog(const std::string& filename);

    /**
     * \brief Function to enable the log
     * This function enable the particular log associated with the object. This function
     * sets the enable attribute to true. The log file is already created when this 
     * function is called, and it is not affected by its call.
     *
     * This function throws an exception if there is any error.
     */
    void enable(void);

    /** 
     * \brief Function to disable the log
     * This function disables the particular log associated with the object. This function
     * sets the enable attribute to false. The log file is already created when this 
     * function is called, and it is not affected by its call.
     *
     * This function throws a CLogException if there is any error.
     */
    void disable(void);

    /** 
     * \brief Function to check if the log is enabled
     *
     * This function checks if the message logging for the corresponding object is enabled
     * or not. 
     *
     * This function throws a CLogException if there is any error.
     *
     * \return The state of the local enable, true if the particular log is enabled or false
     * otherwise.
     *
     */
    bool is_enabled(void);

    /**
     * \brief Function to get the full filename
     *
     * This function returns the full filename of rthe log file including the path and the
     * extension. 
     *
     * This function throws a CLogException if there is any error.
     *
     * \return A null terminated string with the full filename of the log file.
     *
     */ 
    std::string get_filename(void);

    /**
     * \brief Function to set the time format
     *
     * This function sets the time format format to be used in each of the log
     * entries. 
     *
     * \param format the time format to be used. See the documentation on the 
     *               ctimeformat enummeration type for more information.
     *
     */ 
    void set_time_format(ctimeformat format);

    /**
     * \brief Function to get the time format
     *
     * This functions returns the current time format used to generate each of the log 
     * entries in ctimeformat type.
     *
     * \return the current time format used. See the documentation on the ctimeformat 
     *         enummeration type for more information.
     *
     */ 
    ctimeformat get_time_format();
    
    /** 
     * \brief Function to write a variable to the log
     * 
     * This function writes the user variable to the log file together with the time stamp at
     * which the message has been logged. If the global enable is not set, or else, if it is 
     * set but the local enable is not, this function does nothing. The only way to actually 
     * log user messages is that both the local and global enables are set.
     *
     * This function throws a CLogException if there is any error.
     *
     * \param value any kind of basic C++ data types which provides an overloaded version of
     *              the << operator (int, double, std::string, etc.). Vectors are not supported
     *              by this function, use the log_vector() function instead.
     *
     */
    template<class T>
    void log(const T& value)
    {
      this->access.enter();

      this->time_stamp.set();
      if (this->is_enabled())
        this->log_file << "[" << this->time_stamp.getString() << "] - " << value << std::endl;

      this->access.exit();
    }

    /** 
     * \brief Function to write a vector of variable to the log
     * 
     * This function writes the user vector to the log file together with the time stamp at
     * which the message has been logged. If the global enable is not set, or else, if it is 
     * set but the local enable is not, this function does nothing. The only way to actually 
     * log user messages is that both the local and global enables are set.
     *
     * This function throws a CLogException if there is any error.
     *
     * \param values a vector of any kind of basic C++ data types which provides an overloaded 
     *               version of the << operator (int, double, std::string, etc.). 
     *
     */
    template<class T>
    void log_vector(const std::vector<T>& values)
    {
      int i=0;

      this->access.enter();
      this->time_stamp.set();
      if (this->is_enabled())
      {
        this->log_file << "[" << this->time_stamp.getString() << "] - ";
        for(i=0;i<values.size();i++)
        {
          this->log_file << values[i];
          if(i<values.size()-1)
            this->log_file << ",";
        }
        this->log_file << std::endl;
      }
      this->access.exit();
    }


    /** 
     * \brief Destructor
     *
     * This function closes the log file even if it is locally or globally disabled. All the
     * associated memory is freed, but the value of the global enable is not modified
     * because there may exist other CLog objects still created.
     *
     * This function throws a CLogException if there is any error.
     */
    ~CLog();
};

#endif
