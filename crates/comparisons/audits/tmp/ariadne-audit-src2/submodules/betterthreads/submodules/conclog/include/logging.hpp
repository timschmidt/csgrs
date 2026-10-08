/***************************************************************************
 *            logging.hpp
 *
 *  Copyright  2021  Luca Geretti
 *
 ****************************************************************************/

/*
 * This file is part of ConcLog, under the MIT license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CONCLOGGING_HPP
#define CONCLOGGING_HPP

#include <iostream>
#include <fstream>
#include <cstring>
#include <utility>
#include <vector>
#include <queue>
#include <map>
#include <sstream>
#include <thread>
#include <mutex>
#include <memory>

#include "thread_registry_interface.hpp"

#if defined(linux) || defined(__linux) || defined(__linux__)
#define CONCLOG_PRETTY_FUNCTION __PRETTY_FUNCTION__
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define CONCLOG_PRETTY_FUNCTION __FUNCSIG__
#elif defined(darwin) || defined(__darwin) || defined(__darwin__) || defined(__APPLE__)
#define CONCLOG_PRETTY_FUNCTION __PRETTY_FUNCTION__
#else
#define CONCLOG_PRETTY_FUNCTION ""
#endif

// Automatic level increase/decrease in a scope; meant to be used once within a function, at top scope; necessary for print holding.
#define CONCLOG_SCOPE_CREATE auto logscopemanager = LogScopeManager(CONCLOG_PRETTY_FUNCTION);
// Managed level increase/decrease around the function fn; if the function throws, manual decrease of the proper level is required.
#define CONCLOG_RUN_AT(level,fn) Logger::instance().increase_level(level); fn; Logger::instance().decrease_level(level);
// Mute the logger for the function fn; if the function throws, manual decrease of the proper level is required.
#define CONCLOG_RUN_MUTED(fn) Logger::instance().mute_increase_level(); fn; Logger::instance().mute_decrease_level();
// Print one line at the current level; the text shouldn't have carriage returns, but for efficiency purposes this is not checked.
#define CONCLOG_PRINTLN(text) { if (!Logger::instance().is_muted_at(0)) { std::ostringstream logger_stream; logger_stream << std::boolalpha << text; Logger::instance().println(0,logger_stream.str()); } }
// Print one line at an increased level with respect to the current one; the text shouldn't have carriage returns, but for efficiency purposes this is not checked.
#define CONCLOG_PRINTLN_AT(level,text) { if (!Logger::instance().is_muted_at(level)) { std::ostringstream logger_stream; logger_stream << std::boolalpha << text; Logger::instance().println(level,logger_stream.str()); } }
// Print variable in one line at the current level, using the formatting convention.
#define CONCLOG_PRINTLN_VAR(var) { if (!Logger::instance().is_muted_at(0)) { std::ostringstream logger_stream; logger_stream << std::boolalpha << #var << " = " << var; Logger::instance().println(0,logger_stream.str()); } }
// Print variable in one line at the increased level with respect to the current one, using the formatting convention.
#define CONCLOG_PRINTLN_VAR_AT(level,var) { if (!Logger::instance().is_muted_at(level)) { std::ostringstream logger_stream; logger_stream << std::boolalpha << #var << " = " << var; Logger::instance().println(level,logger_stream.str()); } }
// Print a text at the bottom line, holding it until the function scope ends; this requires creation of the scope.
// Nested calls in separate functions append to the held line.
// The text for obvious reasons shouldn't have newlines and carriage returns; for efficiency purposes this is not checked.
#define CONCLOG_SCOPE_PRINTHOLD(text) { if (!Logger::instance().is_muted_at(0)) { std::ostringstream logger_stream; logger_stream << std::boolalpha << text; Logger::instance().hold(CONCLOG_PRETTY_FUNCTION,logger_stream.str()); } }

namespace ConcLog {

using OutputStream = std::ostream;
template<class T> using SharedPointer = std::shared_ptr<T>;
using SizeType = size_t;

//! \brief Exception for trying to use threaded functionality with no attached thread registry
class LoggerNoThreadRegistryException : public std::exception { };
//! \brief Exception for trying to modify the thread registry, which should be immutable as soon as attached
class LoggerModifyThreadRegistryException : public std::exception { };
//! \brief Exception for trying to change the scheduler while there are registered threads
class LoggerSchedulerChangeWithRegisteredThreadsException : public std::exception { };

//! \brief A styling for a text character
//! \details Refer to https://www.lihaoyi.com/post/BuildyourownCommandLinewithANSIescapecodes.html#256-colors
//! for the detailed 256 map of colors for text and background
struct TerminalTextStyle {
    TerminalTextStyle(uint8_t fontcolor, uint8_t bgcolor, bool bold, bool underline);

    //! \brief Return the string code that applies the style
    std::string operator()() const;

    //! \brief If a style is actually set
    bool is_styled() const;

    uint8_t fontcolor; // The color of the character
    uint8_t bgcolor; // The color of the background of the character box
    bool bold; // If the character must be highlighted in bold
    bool underline; // If the character must be underlined

    inline static std::string RESET = "\u001b[0m";
};

static TerminalTextStyle TT_STYLE_NONE(0,0,false,false);
static TerminalTextStyle TT_STYLE_DARKBROWN(95,0,false,false);
static TerminalTextStyle TT_STYLE_DARKORANGE(130,0,false,false);
static TerminalTextStyle TT_STYLE_LIGHTBROWN(136,0,false,false);
static TerminalTextStyle TT_STYLE_BRIGHTORANGE(208,0,false,false);
static TerminalTextStyle TT_STYLE_CREAM(223,0,false,false);
static TerminalTextStyle TT_STYLE_OBSIDIAN(237,0,false,false);
static TerminalTextStyle TT_STYLE_DARKGREY(244,0,false,false);
static TerminalTextStyle TT_STYLE_LIGHTGREY(251,0,false,false);

//! \brief A set of text styles for different elements of the logging interface
struct TerminalTextTheme {
    TerminalTextTheme();
    TerminalTextTheme(TerminalTextStyle level_number, TerminalTextStyle level_shown_separator, TerminalTextStyle level_hidden_separator,
                      TerminalTextStyle multiline_separator, TerminalTextStyle assignment_comparison, TerminalTextStyle miscellaneous_operator,
                      TerminalTextStyle round_parentheses, TerminalTextStyle square_parentheses, TerminalTextStyle curly_parentheses,
                      TerminalTextStyle colon, TerminalTextStyle comma, TerminalTextStyle number, TerminalTextStyle at, TerminalTextStyle keyword);

    TerminalTextStyle level_number; // The number representing the log level
    TerminalTextStyle level_shown_separator; // The separator when the level is shown
    TerminalTextStyle level_hidden_separator; // The separator when the level is hidden

    TerminalTextStyle multiline_separator; // The separator for multiple lines output
    TerminalTextStyle assignment_comparison; // Characters: =, >, <, !
    TerminalTextStyle miscellaneous_operator; // Characters: +, -, *, /, \, ^, |, &, %

    TerminalTextStyle round_parentheses; // Characters: ( )
    TerminalTextStyle square_parentheses; // Characters: [ ]
    TerminalTextStyle curly_parentheses; // Characters: { }

    TerminalTextStyle colon; // Character: :
    TerminalTextStyle comma; // Character: ,
    TerminalTextStyle number; // A 0-9 digit, or a dot after a digit, but not if parsing a numbered_variable

    TerminalTextStyle at; // Character: @
    TerminalTextStyle keyword; // Specific words, not pre/post-fixed by an alphanumeric character: virtual, const, true, false, inf

    bool has_style() const; // Whether at least one style is set

    friend OutputStream& operator<<(OutputStream& os, TerminalTextTheme const& theme);
};

//! \brief Empty theme, for not forcing any theme
static TerminalTextTheme TT_THEME_NONE = TerminalTextTheme();
//! \brief Theme for black background
static TerminalTextTheme TT_THEME_DARK = TerminalTextTheme(TT_STYLE_BRIGHTORANGE,TT_STYLE_BRIGHTORANGE,TT_STYLE_DARKGREY,
                                                           TT_STYLE_DARKGREY,TT_STYLE_BRIGHTORANGE,TT_STYLE_LIGHTBROWN,
                                                           TT_STYLE_DARKORANGE,TT_STYLE_DARKGREY,TT_STYLE_LIGHTBROWN,
                                                           TT_STYLE_LIGHTBROWN,TT_STYLE_DARKGREY,TT_STYLE_CREAM,
                                                           TT_STYLE_DARKGREY,TT_STYLE_DARKGREY);
//! \brief Theme for white background
static TerminalTextTheme TT_THEME_LIGHT = TerminalTextTheme(TT_STYLE_BRIGHTORANGE,TT_STYLE_BRIGHTORANGE,TT_STYLE_DARKGREY,
                                                            TT_STYLE_DARKGREY,TT_STYLE_DARKBROWN,TT_STYLE_DARKBROWN,
                                                            TT_STYLE_DARKGREY,TT_STYLE_OBSIDIAN,TT_STYLE_LIGHTBROWN,
                                                            TT_STYLE_LIGHTBROWN,TT_STYLE_OBSIDIAN,TT_STYLE_DARKORANGE,
                                                            TT_STYLE_DARKGREY,TT_STYLE_DARKGREY);

//! \brief Support class for managing log level increase/decrease in a scope
//! \details Since it is possible to capture the scope string of a function only, nested scopes in a function have the same scope string
//! (but work as expected in terms of level management)
class LogScopeManager {
  public:
    //! \brief Construct with a given scope, and optionally choosing the amount of level increase
    LogScopeManager(std::string scope, unsigned int level_increase=1);
    std::string scope() const;
  public:
    virtual ~LogScopeManager();
  private:
    std::string const _scope;
    unsigned int const _level_increase;
};

//! \brief Supported kinds of log messages
enum RawMessageKind { PRINTLN, HOLD, RELEASE };

//! \brief A log message in raw form, as submitted to a concurrent scheduler
//! \details This does not hold any thread identifier information yet, for efficiency
struct LogThinRawMessage {

    LogThinRawMessage(std::string scope, unsigned int level, std::string text);

    std::string scope;
    unsigned int level;
    std::string text;

    RawMessageKind kind() const;
};

//! \brief Full log message information in raw form, before formatting for actual output
struct LogRawMessage : public LogThinRawMessage {
    LogRawMessage(std::string id, LogThinRawMessage msg) : LogThinRawMessage(msg), identifier(id) { }
    LogRawMessage(std::string id, std::string scp, unsigned int lvl, std::string txt) : LogRawMessage(id,LogThinRawMessage(scp,lvl,txt)) { }
    LogRawMessage(std::string scp, unsigned int lvl, std::string txt) : LogThinRawMessage(scp,lvl,txt), identifier("") { }
    std::string identifier;
};

class LoggerData;

//! \brief The policy for printing the thread id with respect to the log level: NEVER, BEFORE or AFTER
enum class ThreadNamePrintingPolicy { NEVER, BEFORE, AFTER };

OutputStream& operator<<(OutputStream& os, const ThreadNamePrintingPolicy& p);

//! \brief Configuration of visualisation settings for a Logger
class LoggerConfiguration {
  public:

    LoggerConfiguration();

    //! \brief Configuration setters

    //! \brief The depth of printing, where v=0 prevents all printing
    void set_verbosity(unsigned int v);
    //! \brief If true, indents each line with a number of spaces equal to the level
    void set_indents_based_on_level(bool b);
    //! \brief If false, the level is shown on each first line of a print; if true, only when a print has a different level
    void set_prints_level_on_change_only(bool b);
    //! \brief If true, each entrance in a defined scope is logged
    void set_prints_scope_entrance(bool b);
    //! \brief If true, each exit from a defined scope is logged
    void set_prints_scope_exit(bool b);
    //! \brief If true, text that would be printed on multiple lines is correctly preambled on each line with level and indentation
    void set_handles_multiline_output(bool b);
    //! \brief If true, discards from printing all newlines and their immediately following whitespaces
    void set_discards_newlines_and_indentation(bool b);
    //! \brief Decides if and where to append the thread name to the log level (if the latter is shown)
    //! \details The policy is implicitly NEVER if the scheduler is immediate, or only one thread is registered (logging thread excluded)
    void set_thread_name_printing_policy(ThreadNamePrintingPolicy p);

    //! \brief Configuration getters

    unsigned int verbosity() const;
    bool indents_based_on_level() const;
    bool prints_level_on_change_only() const;
    bool prints_scope_entrance() const;
    bool prints_scope_exit() const;
    bool handles_multiline_output() const;
    bool discards_newlines_and_indentation() const;
    ThreadNamePrintingPolicy thread_name_printing_policy() const;

    //! \brief Style theme for terminal output
    void set_theme(TerminalTextTheme const& theme);
    //! \brief Get the current theme used
    TerminalTextTheme const& theme() const;
    //! \brief Add a keyword to the default ones offered, forcing a given style
    //! \details Adding an existing keyword has no effect
    void add_custom_keyword(std::string const& text, TerminalTextStyle const& style);
    //! \brief Add a keyword to the default ones offered, using the keyword style for the current theme
    //! \details Adding an existing keyword has no effect. Changing the theme will not apply to this custom keyword.
    void add_custom_keyword(std::string const& text);
    //! \brief Get the map of keywords (a TerminalTextStyle equal to TT_STYLE_NONE implies no custom style forced)
    std::map<std::string,TerminalTextStyle> const& custom_keywords() const;

    friend OutputStream& operator<<(OutputStream& os, LoggerConfiguration const& configuration);

  private:
    unsigned int _verbosity;
    bool _indents_based_on_level;
    bool _prints_level_on_change_only;
    bool _prints_scope_entrance;
    bool _prints_scope_exit;
    bool _handles_multiline_output;
    bool _discards_newlines_and_indentation;
    ThreadNamePrintingPolicy _thread_name_printing_policy;

    TerminalTextTheme _theme;
    std::map<std::string,TerminalTextStyle> _custom_keywords;
};

class LoggerSchedulerInterface;

//! \brief A static class for log output handling.
//! Configuration and final printing is done here, while scheduling is
//! done in a separate class.
class Logger {
  private:
    friend class ImmediateLoggerScheduler;
    friend class BlockingLoggerScheduler;
    friend class NonblockingLoggerScheduler;

    Logger();
  public:
    ~Logger();
    Logger(Logger const&) = delete;
    void operator=(Logger const&) = delete;

    static Logger& instance() {
        static Logger instance;
        return instance;
    }

    //! \brief Attach a thread registry, necessary to modify the scheduler and register/unregister
    void attach_thread_registry(ThreadRegistryInterface* registry);
    //! \brief Check whether there is already a thread registry attached
    bool has_thread_registry_attached() const;

    void use_immediate_scheduler();
    void use_blocking_scheduler();
    void use_nonblocking_scheduler();

    void redirect_to_file(const char* filename);
    void redirect_to_console();

    void register_thread(std::thread::id id, std::string name);
    void unregister_thread(std::thread::id id);
    //! \brief Registers this same thread with a specific \a name and \a level
    //! \details Required when the registering thread is the same as the one to register,
    //! since the level cannot be obtained
    void register_self_thread(std::string name, unsigned int level);

    void println(unsigned int level_increase, std::string text);
    void hold(std::string scope, std::string text);
    void release(std::string scope);

    void increase_level(unsigned int i);
    void decrease_level(unsigned int i);
    void mute_increase_level();
    void mute_decrease_level();

    bool is_muted_at(unsigned int i) const;

    unsigned int current_level() const;
    std::string current_thread_name() const;
    std::string cached_last_printed_thread_name() const;

    unsigned int get_window_columns() const;

    LoggerConfiguration& configuration();

  private:
    std::string _apply_theme(std::string const& text) const;
    std::string _apply_theme_for_keywords(std::string const& text) const;
    void _print_preamble_for_firstline(unsigned int level, std::string thread_name);
    void _print_preamble_for_extralines(unsigned int level);
    std::string _discard_newlines_and_indentation(std::string const& text);
    void _cover_held_columns_with_whitespaces(unsigned int printed_columns);
    void _print_held_line();
    void _println(LogRawMessage const& msg);
    void _hold(LogRawMessage const& msg);
    void _release(LogRawMessage const& msg);
    bool _is_holding() const;
    bool _can_print_thread_name() const;
  private:
    static const unsigned int _MUTE_LEVEL_OFFSET;
    static const std::string _MAIN_THREAD_NAME;
    std::ofstream _redirect_file;
    std::basic_streambuf<char>* _default_streambuf;
    std::vector<LogRawMessage> _current_held_stack;
    unsigned int _cached_num_held_columns;
    unsigned int _cached_last_printed_level;
    std::string _cached_last_printed_thread_name;
    std::shared_ptr<LoggerSchedulerInterface> _scheduler;
    ThreadRegistryInterface* _thread_registry;
    LoggerConfiguration _configuration;
};

} // namespace ConcLog

#endif // CONCLOGGING_HPP
