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

#ifndef CTime_H
#define CTime_H

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include "limits.h"
#include <sys/time.h>
#include "ctimeexceptions.h"

/**
 * \brief supported time formats
 *
 * This enummeration holds an identifier for each of the supported formats 
 * for time. The currently supported formats are:
 *
 * * seconds and nanoseconds.
 *
 * * string with data and time
 *
 * * miliseconds
 */
enum ctimeformat { ctf_secnano, ctf_datetime, ctf_ms };

/** \class CTime
  * 
  * \brief Implementation of a time class and operations
  *  
  * This class is designed to easily handle time within a program. It provides
  * to the user a friendly interface in miliseconds and allows him to get the
  * current time or convert any given time in any of the system dependant 
  * structures.
  *
  * Each CTime object has a time since the 
  * <a href="http://en.wikipedia.org/wiki/Epoch_(reference_date)">Epoch</a>. 
  * This class can also work with user provided times which are not referenced 
  * to the previous zero time, but can be used as timeouts or to monitor
  * time between events.
  *
  * Also, given a time in any format it provides a set of static functions 
  * to convert from that format to any other format in order to simplify the 
  * time handling issues without having to create an object of this class.
  *
  * Finally, some basic arithmetic operators are provided (addition, 
  * substraction, ..) as well as relational operators are provided to 
  * perform basic operations on objects of this class.
  *
  * \image html http://imgs.xkcd.com/comics/bug.png 
  *
  * \author Martí Morta (mmorta@iri.upc.edu)
  */

class CTime
{
  private:
    /**
     * \brief Internal time (seconds)
     *
     * This attribute holds the assigned time in seconds. This value is 
     * initialized at construction time or whenever the set() function
     * is called. When any of the get functions is called, its value is
     * used to generate the desired output.
     *
     * This value is always positive since it represents time, but when 
     * it is the result of an arithmetic operation, its value can be
     * negative.
     *
     * This attribute is not used in the conversion functions since 
     * they are static.
     */ 
    long sec;
    /**
     * \brief Internal time (nanoseconds)
     *
     * This attribute holds the assigned time in nanoseconds. This value is 
     * initialized at construction time or whenever the set() function
     * is called. When any of the get functions is called, its value is
     * used to generate the desired output.
     *
     * This value is always positive since it represents time, but when 
     * it is the result of an arithmetic operation, its value can be
     * negative.
     *
     * This attribute is not used in the conversion functions since 
     * they are static.
     */ 
    long nsec;
    /**
     * \brief current output format
     *
     * This attribute holds the desired time format used by the << operator
     * to show the time on screen or print it to a file.
     */ 
    ctimeformat print_format;    
  public:

    /** \brief Constructor
      *
      * This constructor creates a new object of this class initialized with
      * the current time. 
      *
      */
    CTime();

    /** \brief Constructor with parameters
      *
      * This constructor creates a new object of this class initialized with
      * the value specified by the user. This value is a relative time in 
      * miliseconds.
      *
      * \param relative the desired relative time in miliseconds. This value must 
      *                 be always positive since it represents time.
      */
    CTime(double relative); 
    
    
    /** 
     * \brief Destructor 
     *
     * This destructor frees all the resources associated with the object.
     */
    ~CTime();

  /// @name Get time functions
  /// @{

    /** 
      * \brief Get time in seconds 
      *
      * This function returns the internal time in seconds with milliseconds 
      * resolution. 
      *
      * \return the internal time in seconds with milliseconds resolution (only
      *         the first 3 decimal places have real meaning)
      *
      */
    double getTimeInSeconds(void);

    /** \brief Get time in milliseconds
      *
      * This function returns the internal time in milliseconds with microseconds
      * resolution.
      *
      * \return the internal time in milliseconds with microseconds resolution (only
      *         the first 3 decimal places have real meaning)
      *
      */
    double getTimeInMilliseconds(void);

    /** \brief Get time in microseconds
      * 
      * This function returns the internal time in microseconds as an integer.
      *
      * \return the internal time in microseconds.
      *
      */
    long getTimeInMicroseconds(void);

    /** \brief Get time in a timespec strcuture
      *
      * This function returns the internal time using a timespec Linux structure. 
      * This may prove usefulll when using some Linux system functions which
      * require this kind of structure.
      *
      * The format of the timespec structure is as follows:
      * \code 
      * time_t tv_sec 
      * long   tv_nsec 
      * \endcode
      *
      * \return the internal time within a timespec strucure.
      */
    timespec getTimeInTimespec(void);

    /** \brief Get time in a timeval strcuture
      *
      * This function returns the internal time using a timeval Linux structure. 
      * This may prove usefulll when using some Linux system functions which
      * require this kind of structure.
      *
      * The format of the timeval structure is as follows:
      * \code 
      * time_t      tv_sec s
      * suseconds_t tv_usec 
      * \endcode
      *
      * \return the internal time within a timeval strucure.
      */
    timeval getTimeInTimeval(void);

    /**
     * \brief Get time in time_t
     *
     * This function returns the internal time as a time_t strcuture.
     * This may prove usefulll when using some Linux system functions which
     * require this kind of structure.
     *
     * return the internal time as a time_t structure
     */ 
    time_t getTimeInTime_t(void);
  ///@}

  /// @name Set Time
  /// @{

    /** \brief Sets the internal time
      *
      * This function sets the internal time of the object once it is already
      * created. If no parameter is passed (or a negative value is provided), 
      * the current computer time is used. Otherwise, the time provided by the
      * user in milliseconds is used to initialize the internal time.
      *
      * \param milliseconds the desired time in milliseconds to initialize the
      *                     object. If this parameter is not provided or it has
      *                     a negative value, the current computer time will be 
      *                     used.
      *
      */
    void set(double milliseconds=-1.0);

  ///@}

  /// @name Operators
  /// @{

    /** \brief Calculates time difference
      *
      * This operator computes the difference between two time objects. The
      * resulting time object may have a negative value because it represents
      * a difference.
      *
      * \param t a second time object used to perform the desired arithmetic
      *          operation.
      *
      * \return a new object initialized with the time difference between the
      *         object itself and the one provided as an argument.
      *
      *         \code
      *         return = this - t2
      *         \endcode
      *
      */   
    CTime operator - (CTime &t);

    /** \brief Calculates time addition
      *
      * This operator computes the addition of two time objects. Depending on 
      * the values of the time objects to add, it is possible the resulting
      * value overflows the internal attributes. In this case an exception is 
      * thrown.
      *
      * \param t a second time object used to perform the desired arithmetic
      *          operation.
      *
      * \return a new object initialized with the time addition of the
      *         object itself and the one provided as an argument.
      *
      *         \code
      *         return = this + t
      *         \endcode
      * 
      */
    CTime operator + (CTime &t);
    
    /** \brief Calculates division by integer
     *
     * This operator computes the division of the time object by an integer value.
     * It can be usefull when computing averages of time objects. The value of the
     * resulting time object is rounded to the nanoseconds.
     *
     * \param div an integer value used to dived the internal time value. This value
     *            can be negative, in which case the resulting time object will also
     *            have a negative value.
     *
     * return a new object initialized with the time of the object itself divided
     *        by the provided integer.
     */
    CTime operator / (int div);

    /** \brief Comparison operator
      *
      * This operator is used to compare two time objects. If the two values are
      * the same it returns true, and false otherwise.
      *
      * \param t a second time object used to perform the comparation. 
      *
      * \return returns true if the two objects have the same value and false
      *         otherwise.
      */
    bool operator == (CTime &t);

    /** \brief outputs time into a ostream
      *
      * This function is used to output formatted time information into a ostream
      * object, such as the standard output or a file. This function uses the 
      * current value of the print_format attribute to provided teh desired output
      * format. 
      *
      * Use the setFormat() and getFormat() functions to change and check the 
      * current output format respectively. See the documentation on the 
      * ctimeformat enummeration for information on the supported formats.
      *
      * \param o the ostream object to ouput the formatted information. This 
      *          can be the standard output or a file.
      *
      * \param t a time object whose time information is to be formatted and
      *          output into the ostream obejct.
      *
      * \return the same ostream object passed as an argument.
      */
    friend std::ostream& operator << (std::ostream &o, CTime &t);

    /** \brief Gives a formatted string of this time
      *
      * This function is similar to the << operator, but it returns a formatted
      * string instead of outputting it into an ostream object. In this case, the
      * output format is also determined by the current value of the print_format
      * attribute.
      *
      * Available formats are:
      *
      * Seconds Nanoseconds ( \b ctf_secnano ) 
      * \code
      * 1265731752 81195677
      * \endcode
      *
      * Date,Time ( \b ctf_datetime )
      * \code
      * 2010-02-09,17:09:12 
      * \endcode
      *
      * Milliseconds ( \b ctf_ms )
      * \code
      * 1265731752.812
      * \endcode
      *
      * \return a string with the time information in the desired format.
      *
      */
    std::string getString(void);
    
    /** \brief Sets output string format
      *
      * This function sets the desired ouptu format for the getString()
      * function and the << operator.
      *
      * \param format an identifier of the desired format. It must belong to
      *               the ctimeformat enummeration type. Currently, the only 
      *               supported formats are:
      *
      *               * ctf_secnano -> for seconds and nanoseconds
      *               * ctf_datatime -> for a strign with the human redable 
      *                                 date and time
      *               * ctf_ms -> for miliseconds
      *
      *               For more information see the documentation on the 
      *               ctimeformat enummeration type.
      */
    void setFormat(ctimeformat format);
    
   /** \brief Gets output string format
     *
     * This function return the current output format being used in the
     * getString() function and the << operator. 
     *
     * \return the current output format, which is one of the ctimeformat
     *         enummeration type. See the documentation on that type for
     *         more information.
     */
    ctimeformat getFormat();
    
  ///@} 

  /// @name Conversion functions
  /// @{

    /** \brief conversion from timespec to milliseconds
     *
     * This function converts a time in a timespec structure into a
     * double value with the same time in miliseconds. This function is
     * static, so it can be used even if there exist no object of this
     * class.
     *
     * \param t a valid timespec structure already initialized.
     *
     * \return the time in the timespec structure represented in milliseconds.
     */
    static double timespecToMs(timespec t);

    /** \brief conversion from milliseconds to timespec
      *
     * This function converts a time in milliseconds into a timespec
     * structure (seconds and nanoseconds). This function is
     * static, so it can be used even if there exist no object of this
     * class.
     *
      * \param ms the time in milliseconds.
      * \return a new timespec structure initialized with the time
      *         passed as argument.
      */
    static timespec msToTimespec(double ms);

  ///@} 


};

#endif

