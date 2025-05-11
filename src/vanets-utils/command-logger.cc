#include "command-logger.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE("CommandLogger");

/**
 * \brief Checks if a file doesn't exist or is empty
 * \param path Path to the file
 * \return true if the file doesn't exist or is empty, false otherwise
 */
bool IsFileEmptyOrNotExists(const boost::filesystem::path& path) {
    NS_LOG_FUNCTION(path);
    return (!boost::filesystem::exists(path) || boost::filesystem::file_size(path) == 0);
}

/**
 * \brief Writes the executed command to a file
 * \param path Path to the log file
 * \param argc Argument count from main
 * \param argv Argument values from main
 */
void WriteExecutedCommand(const boost::filesystem::path& path, int argc, char* argv[]) {
    NS_LOG_FUNCTION(path << argc);

    NS_LOG_INFO("Logging command to: " << path.string());

    // Ensure the parent directories exist
    boost::filesystem::create_directories(path.parent_path());

    std::ofstream out(path.string(), std::ios::out);  // Overwrites if file exists

    if (!out) {
        NS_LOG_ERROR("Failed to open file at: " << path.string());
        return;
    }

    // Extract the executable name (argv[0] without full path)
    std::string commandName = boost::filesystem::path(argv[0]).filename().string();

    // Write the command name
    out << commandName;

    // Write the rest of the arguments
    for (int i = 1; i < argc; ++i) { // Start from 1 to skip argv[0]
        out << " " << argv[i];
    }

    out << std::endl;
    out.close();

    NS_LOG_INFO("Executed command successfully written to: " << path.string());
}
