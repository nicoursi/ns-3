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

#include "csv-manager.h"
#include "ns3/log.h"
#include <sys/time.h>
#include <iostream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("CSVManager");

CSVManager::CSVManager ()
  : m_csvFilePath (""),
  runId (0)
{
  NS_LOG_FUNCTION (this);
}

CSVManager::CSVManager (unsigned int runId)
  : m_csvFilePath (""),
  runId (runId)
{
  NS_LOG_FUNCTION (this);
}

CSVManager::~CSVManager ()
{
  NS_LOG_FUNCTION (this);
}

void
CSVManager::Setup (std::string filename)
{
  NS_LOG_FUNCTION (this);
  m_csvFilePath = filename;
}

void
CSVManager::WriteHeader (std::string header)
{
  NS_LOG_FUNCTION (this);

  boost::filesystem::path parentPath = m_csvFilePath.parent_path ();
  boost::filesystem::create_directories (parentPath);
  std::ofstream out (m_csvFilePath.string ());
  if (!out.is_open ())
    {
      NS_LOG_ERROR ("Failed to open file for writing header: " << m_csvFilePath.string ());
      return;
    }
  out << header.c_str () << std::endl;
  out.close ();
}

std::string
CSVManager::GenerateRandomTag ()
{
  static const char charset[] = "abcdefghijklmnopqrstuvwxyz"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "0123456789";
  std::default_random_engine rng (std::random_device{} ());
  std::uniform_int_distribution<> dist (0, sizeof (charset) - 2);
  std::string tag;
  for (int i = 0; i < 4; ++i)
    {
      tag += charset[dist (rng)];
    }
  return tag;
}

void
CSVManager::EnableAlternativeFilename (boost::filesystem::path path)
{
  NS_LOG_FUNCTION (this);
  std::string extension = ".csv";

  // Get unix time
  struct timeval tp;
  gettimeofday (&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

  // Create the new filename
  std::string finalPartOfPath = "-" + std::to_string (ms) + "-" + GenerateRandomTag () + extension;
  m_csvFilePath = path;
  m_csvFilePath += finalPartOfPath;
  std::cout << "CSV File Path = " << m_csvFilePath << std::endl;
}

void
CSVManager::AddValue (std::stringstream value)
{
  NS_LOG_FUNCTION (this);
  m_currentRow << value.str () << ",";
}

void
CSVManager::AddValue (int value)
{
  NS_LOG_FUNCTION (this);
  m_currentRow << value << ",";
}

void
CSVManager::AddValue (double value)
{
  NS_LOG_FUNCTION (this);
  m_currentRow << value << ",";
}

void
CSVManager::AddValue (std::string value)
{
  NS_LOG_FUNCTION (this);
  m_currentRow << value << ",";
}

void
CSVManager::AddMultipleValues (std::stringstream& value)
{
  NS_LOG_FUNCTION (this);
  m_currentRow << value.str ();
}

void
CSVManager::SetRunId (unsigned int id)
{
  NS_LOG_FUNCTION (this << id);
  runId = id;
}

void
CSVManager::CloseRow (void)
{
  NS_LOG_FUNCTION (this);
  // write current row to file in an atomic way
  std::ofstream out (m_csvFilePath.c_str (), std::ios::app);
  if (!out.is_open ())
    {
      NS_LOG_ERROR ("Failed to open file for writing row: " << m_csvFilePath.c_str ());
      return;
    }
  out << runId << "," << m_currentRow.str () << std::endl;
  out.close ();
  // Delete (old) row
  m_currentRow.str ("");
}

bool
CSVManager::IsFileEmptyOrNonexistent (const boost::filesystem::path& path)
{
  NS_LOG_FUNCTION (path.string ());

  // Check if the file exists
  if (!boost::filesystem::exists (path))
    {
      NS_LOG_INFO ("File does not exist: " << path.string ());
      return true;
    }

  // Check if the file is empty
  if (boost::filesystem::file_size (path) == 0)
    {
      NS_LOG_INFO ("File is empty: " << path.string ());
      return true;
    }

  return false;
}

} // namespace ns3
