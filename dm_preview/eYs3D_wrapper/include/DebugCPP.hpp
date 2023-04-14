#pragma once
#include <cstdio>
#include <string>
#include <sstream>

#include <memory>
#include <string>
#include <stdexcept>

// Reference from https://stackoverflow.com/questions/43732825/use-debug-log-from-c
#ifdef UNITY_DEBUG_LOG
#if defined(_MSC_VER) 	//  Microsoft 
	#define EXPORT __declspec(dllexport)
	#define IMPORT __declspec(dllimport)
#elif defined(__GNUC__)	//  GCC
	#define EXPORT __attribute__((visibility("default")))
	#define IMPORT
#else 	// do nothing and hope for the best?
	#define EXPORT
	#define IMPORT
	#pragma warning Unknown dynamic link import/export semantics.
#endif

#define LOGW(...) do {Debug::Log(Debug::string_format(__VA_ARGS__), Color::Orange);} while (0);
#define LOGV(...) do {Debug::Log(Debug::string_format(__VA_ARGS__), Color::Black);} while (0);
#define LOGI(...) do {Debug::Log(Debug::string_format(__VA_ARGS__), Color::Black);} while (0);
#define LOGE(...) do {Debug::Log(Debug::string_format(__VA_ARGS__), Color::Red);} while (0);
#define LOGD(...) do {Debug::Log(Debug::string_format(__VA_ARGS__), Color::Black);} while (0);



extern "C"
{
    //Create a callback delegate
    typedef void(*FuncCallBack)(const char* message, int color, int size);
    static FuncCallBack callbackInstance = nullptr;
    void RegisterDebugCallback(FuncCallBack cb);
}

//Color Enum
enum class Color { Red, Green, Blue, Black, White, Yellow, Orange };

class  Debug
{
public:
    static void Log(const char* message, Color color = Color::Black);
    static void Log(const std::string message, Color color = Color::Black);
    static void Log(const int message, Color color = Color::Black);
    static void Log(const char message, Color color = Color::Black);
    static void Log(const float message, Color color = Color::Black);
    static void Log(const double message, Color color = Color::Black);
    static void Log(const bool message, Color color = Color::Black);
    template<typename ... Args>
    static std::string string_format( const std::string& format, Args ... args )
    {
        size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
        std::unique_ptr<char[]> buf( new char[ size ] ); 
        snprintf( buf.get(), size, format.c_str(), args ... );
        return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }

private:
    static void send_log(const std::stringstream &ss, const Color &color);
};
#else
    #define LOG_TAG "eys_wrapper"
    #define LOGW(...) fprintf(stderr, "\nW/" LOG_TAG": " __VA_ARGS__)
    #define LOGV(...) fprintf(stderr, "\nV/" LOG_TAG": " __VA_ARGS__)
    #define LOGI(...) fprintf(stderr, "\nI/" LOG_TAG": " __VA_ARGS__)
    #define LOGE(...) fprintf(stdout, "\nE/" LOG_TAG ": " __VA_ARGS__ );
    #define LOGD(...) fprintf(stderr, BLUE "D/" LOG_TAG": " __VA_ARGS__);
#endif
