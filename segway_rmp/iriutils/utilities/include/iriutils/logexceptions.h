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

#ifndef _LOG_EXCEPTIONS
#define _LOG_EXCEPTIONS

#include "exceptions.h"

class CLogException : public CException
{
  public:
    /**
     * \brief Class constructor
     *
     * The constructor calls the base class constructor to add the general
     * exception identifier and then adds the class identifier string 
     * "[Log class]" and the supplied error message. 
     *
     * It also appends the filename of the log file that generated the 
     * exception. So, the total exception message will look like this:
     *
     * \verbatim
     * [Exception caught] - <where>
     * Error: [CLog class] - <error message> - <filename>
     * \endverbatim
     *
     * \param where a null terminated string with the information about the name
     *              of the function, the source code filename and the line where
     *              the exception was generated. This string must be generated 
     *              by the _HERE_ macro.
     *
     * \param error_msg a null terminated string that contains the error message.
     *                  This string may have any valid character and there is no 
     *                  limit on its length.
     *
     * \param filename  a null terminated string that contains the name of the log file
     *                  that generated the exception. This string may have any valid 
     *                  character and there is no limit on its length.
     */
    CLogException(const std::string& where,const std::string& error_msg,const std::string& filename);
};

#endif
