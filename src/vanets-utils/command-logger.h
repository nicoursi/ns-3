#ifndef COMMAND_LOGGER_H
#define COMMAND_LOGGER_H

#include <string>
#include <boost/filesystem.hpp>

/**
 * \brief Check if a file is empty or doesn't exist
 *
 * \param path The Boost path to check.
 * \return true if the file doesn't exist or is empty.
 */
bool
IsFileEmptyOrNotExists (const boost::filesystem::path& path);

/**
 * \brief Write the executed command-line arguments to the given path
 *
 * \param path The Boost path where the command will be logged.
 * \param argc The number of arguments.
 * \param argv The argument list.
 */
void
WriteExecutedCommand (const boost::filesystem::path& path, int argc, char* argv[]);

#endif // COMMAND_LOGGER_H
