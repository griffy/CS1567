/**
 * logger.h
 * 
 * @brief 
 *      This class defines the project logger. It is a singleton that can be
 *      accessed via the LOG macro to intelligently write to files and screen
 *      according to importance level.
 *
 * @author
 *      Shawn Hanna
 *      Tom Nason
 *      Joel Griffith
 *
 **/

#ifndef CS1567_LOGGER_H
#define CS1567_LOGGER_H

#include <map>
#include <string>
#include <cstdarg>

// some default importance levels
#define LOG_OFF 99
#define LOG_HIGH 2
#define LOG_MED 1
#define LOG_LOW 0

// A macro to return the static instance from any file
#define LOG Logger::getInstance()

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }
    void setImportanceLevel(int importanceLevel);
    // write is the same as printf except it adds a newline
    void write(int level, std::string filename, const char *formatString, ...);
    void printf(int level, std::string filename, const char *formatString, ...);
    void printfScreen(int level, std::string filename, const char *formatString, ...);
    void printfFile(int level, std::string filename, const char *formatString, ...);
    void flushFile(std::string filename);
    void flushScreen(std::string filename);
private:
    Logger();
    ~Logger();
    Logger(Logger const&);
    void operator=(Logger const&);

    void _vprintf(int level, std::string filename, const char *formatString, va_list listPointer);
    void _vprintfScreen(int level, std::string filename, const char *formatString, va_list listPointer);
    void _vprintfFile(int level, std::string filename, const char *formatString, va_list listPointer);

    int _importanceLevel;
    std::map<std::string, FILE*> _files;
};

#endif
