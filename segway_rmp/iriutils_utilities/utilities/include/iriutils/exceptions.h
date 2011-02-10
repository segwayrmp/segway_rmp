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

#ifndef _EXCEPTIONS
#define _EXCEPTIONS

#include <exception>
#include <sstream>
#include <string>

template <typename T_> 
inline std::string tostring (const T_ & src)
{
  std::ostringstream out;

  out << src;
  return out.str();
}

#define _HERE_ (std::string(__PRETTY_FUNCTION__) + " at " + \
        std::string(__FILE__) + ":" +tostring(__LINE__))

/** 
 * \brief Generic exception
 * 
 * This class inherits from the std::exception class to provide generic 
 * exceptions for for all classes. The error message is provided at
 * construction time with all the necessary information about the error
 * that has generated the exception. When the exception is caught, the 
 * what() function may be used to get the error message. 
 *
 * This class should be used as a base class for all other exception
 * classes in any program. This way it will be simple to distinguish 
 * between system exceptions and application exceptions.
 *
 * When throwing exceptions, it is important to free all allocated 
 * resources and also to set all the class attributes and variables to a 
 * known state. Otherwise the behavior of the program after an exception
 * is thrown could be unpredictible.
 *
 * Next there is a scketch of a program to catch exceptions:
 *
 * \verbatim
 *   try{
 *     user code here
 *   }catch(CException &e){
 *     handle the exception
 *   }
 * \endverbatim
 *
 * There exist no copy constructor, so if it is necessary to rethrow an
 * exception just use throw; instead of throw e;, because the later tries
 * to create a new CException object and initialize it with the information
 * of the original one.
 *
 * This code will catch any exception of this class or any inherited class.
 * Several catch statements can be used to catch exceptions of different 
 * classes separatelly and handle them in different ways.
 *
 */
class CException : public std::exception
{
  protected:
    /**
     * \brief Exception error message.
     *
     * This attribute is a string which contains the error message with the
     * information about the error that has generated the exception. This 
     * message is allocated and initialized at construction time and can not
     * be modified afterwards.
     *
     * When the exception is caught, the what() function must be used to get
     * the contents of this error message.
     */
    std::string error_msg;
  public:
    /**
     * \brief Class constructor
     *
     * The constructor of this class allocates the necessary memory to store 
     * the error message. In addition to the error message provided to the 
     * constructor, the name of the function, filename and line where the error
     * has ocurred, as well as the text "[Exception caught] - " is pre-appended 
     * to label the message as an exception.
     *
     * \param where a null terminated string with the information about the name
     *              of the function, the source code filename and the line where
     *              the exception was generated. This string must be generated 
     *              by the _HERE_ macro.
     *
     * \param error_msg a null terminated string that contains the error message.
     *                  This string may have any valid character and there is no 
     *                  limit on its length.
     */
    CException(const std::string& where, const std::string& error_msg);
    /**
     * \brief Function to get the error message
     *
     * This function is used to get the error message with the information about 
     * the error that has generated the exception. 
     *
     * \return the error message as a constant string to prevent external
     *          modifications of the private attributes of the class.
     */   
    virtual const std::string& what(void);
    /**
     * \brief Class destructor
     *
     * This destructor just frees all the memory allocated to store the error 
     * message.
     */  
    virtual ~CException() throw();
};

#endif
