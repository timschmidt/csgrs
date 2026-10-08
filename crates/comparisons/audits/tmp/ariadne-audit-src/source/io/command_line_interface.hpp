/***************************************************************************
 *            io/command_line_interface.hpp
 *
 *  Copyright  2007-21  Luca Geretti
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

/*! \file io/command_line_interface.hpp
 *  \brief Support for interacting with the command line.
 */

#ifndef ARIADNE_COMMAND_LINE_INTERFACE_HPP
#define ARIADNE_COMMAND_LINE_INTERFACE_HPP

#include <functional>
#include <queue>
#include <functional>
#include "utility/macros.hpp"
#include "utility/handle.hpp"
#include "utility/container.hpp"
#include "utility/string.hpp"

namespace Ariadne {

using VoidFunction = std::function<Void()>;

//! \brief Exception for when no value is available, but it should be supplied
class MissingArgumentValueException : public std::runtime_error {
public:
    MissingArgumentValueException(String argument) : std::runtime_error(argument) { }
};
//! \brief Exception for when a parser recognises the value argument as invalid
class InvalidArgumentValueException : public std::runtime_error {
  public:
    InvalidArgumentValueException(String argument) : std::runtime_error(argument) { }
};

//! \brief Stream of arguments to be consumed by parsers
class ArgumentStream {
  public:
    //! \brief Construct from a list of String
    ArgumentStream(List<String> const& args);

    //! \brief Peek the head of the stream
    String peek() const;

    //! \brief Extract the head of the stream
    String pop();

    //! \brief If the stream is empty
    Bool empty() const;

    //! \brief Number of the elements in the stream
    SizeType size() const;

  private:
    std::queue<String> _args;
};

//! \brief A pack for an argument with its \a id and the \a processor to process the input
class ArgumentPack {
public:
    ArgumentPack(String const& id, VoidFunction const& processor);
    String const& id() const;
    Void process() const;
    Bool operator<(ArgumentPack const& other) const;
private:
    String const _id;
    VoidFunction const _processor;
};

//! \brief Interface for a parser for a given CLI argument
class ArgumentParserInterface {
  public:
    //! \brief If the tip of the stream is consumable by this parser
    //! \details Checks only the kind, not the value
    virtual Bool is_consumable(ArgumentStream const& stream) const = 0;

    //! \brief Consume the stream, returning a pack of the arguments for processing
    //! \throws InvalidArgumentValueException if the value is incorrect
    //! \details Assumes that it has already been checked by is_consumable
    virtual ArgumentPack consume(ArgumentStream& stream) const = 0;

    //! \brief The size in characters of the help description header for the argument
    virtual SizeType help_description_header_size() const = 0;

    //! \brief The description to print for the help
    //! \details Takes \a num_chars_to_separate_instructions as the number of spaces that
    //! separate the instructions in the help summary for the argument
    virtual String help_description(SizeType num_chars_to_separate_instructions) const = 0;
};

//! \brief An argument parser
class ArgumentParser : public Handle<const ArgumentParserInterface> {
  public:
    using Handle<const ArgumentParserInterface>::Handle;
    Bool is_consumable(ArgumentStream const& stream) const { return this->_ptr->is_consumable(stream); }
    ArgumentPack consume(ArgumentStream& stream) const { return this->_ptr->consume(stream); }
    SizeType help_description_header_size() const { return this->_ptr->help_description_header_size(); }
    String help_description(SizeType num_chars_to_separate_instructions) const { return this->_ptr->help_description(num_chars_to_separate_instructions); }
};

//! \brief A static class for acquisition of CLI arguments
class CommandLineInterface {
  private:
    CommandLineInterface();
    void _print_help() const;
  public:
    CommandLineInterface(CommandLineInterface const&) = delete;
    Void operator=(CommandLineInterface const&) = delete;

    static CommandLineInterface& instance() {
        static CommandLineInterface instance;
        return instance;
    }

    //! \brief Acquire the CLI arguments
    //! \details Returns whether the acquisition was a success and
    //! any subsequent code should be run
    Bool acquire(int argc, const char* argv[]) const;

    //! \brief Acquire from a list of strings
    Bool acquire(List<String> const& args) const;

  private:
    List<ArgumentParser> const _parsers;
};

} // namespace Ariadne

#endif // ARIADNE_COMMAND_LINE_INTERFACE_HPP
