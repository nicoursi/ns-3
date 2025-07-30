/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2025 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef CSV_MANAGER_H
#define CSV_MANAGER_H

#include "ns3/core-module.h"
#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <random>

namespace ns3 {

/**
 * \ingroup vanets-utils
 * \brief A utility class for managing CSV files
 *
 * CSVManager helps with creating and writing to CSV files.
 * It provides methods for setting up the file, writing headers,
 * adding values to rows, and closing rows.
 */
class CSVManager
{
public:
  /**
   * \brief Default constructor
   * \return none
   */
  CSVManager ();

  /**
   * \brief Constructor with runId
   * \param runId ID of the current run
   */
  CSVManager (unsigned int runId);

  /**
   * \brief Destructor
   * \return none
   */
  ~CSVManager ();

  /**
   * \brief Set the filename of the csv file
   * \param filename path of the file
   * \return none
   */
  void Setup (std::string filename);

  /**
   * \brief Write the header of the csv
   * \param header header of the csv
   * \return none
   */
  void WriteHeader (std::string header);

  /**
   * \brief generates a random string to append to the csv filename
   * \return a random string of 4 characters
   */
  std::string GenerateRandomTag ();

  /**
   * \brief Create a new filename adding a timestamp to a provided base
   * \param path The path to use as base for the alternative filename
   * \return none
   */
  void EnableAlternativeFilename (boost::filesystem::path path);

  /**
   * \brief Add a value (cell) in the current row
   * \param value int value to be written
   * \return none
   */
  void AddValue (int value);

  /**
   * \brief Add a value (cell) in the current row
   * \param value double value to be written
   * \return none
   */
  void AddValue (double value);

  /**
   * \brief Add a value (cell) in the current row
   * \param value string value to be written
   * \return none
   */
  void AddValue (std::string value);

  /**
   * \brief Add a value (cell) in the current row
   * \param value stream value to be written
   * \return none
   */
  void AddValue (std::stringstream value);

  /**
   * \brief Add multiple values in the current row
   * \param value stream to be written
   * \return none
   */
  void AddMultipleValues (std::stringstream& value);

  /**
   * \brief Set the run ID manually
   * \param id The run ID to set
   * \return none
   */
  void SetRunId (unsigned int id);

  /**
   * \brief Write the current row and initialize a new one
   * \return none
   */
  void CloseRow (void);

  /**
   * \brief Check if a file is empty or doesn't exist
   *
   * \param path The Boost path to check.
   * \return true if the file doesn't exist or is empty.
   */
  static bool IsFileEmptyOrNonexistent (const boost::filesystem::path& path);

private:
  boost::filesystem::path m_csvFilePath; //!< Path to the CSV file
  std::stringstream m_currentRow;        //!< Current row being built
  unsigned int runId;                    //!< ID of the current run
};

} // namespace ns3

#endif /* CSV_MANAGER_H */
