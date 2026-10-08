/***************************************************************************
 *            io/command_line_interface.cpp
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

#include "config.hpp"
#include "betterthreads/thread_manager.hpp"
#include "utility/handle.hpp"
#include "drawer.hpp"
#include "graphics_manager.hpp"
#include "gnuplot.hpp"
#include "cairo.hpp"
#include "null_graphics.hpp"
#include "conclog/logging.hpp"
#include "command_line_interface.hpp"

using namespace ConcLog;

namespace Ariadne {

using BetterThreads::ThreadManager;


ArgumentStream::ArgumentStream(List<String> const& args) {
    ARIADNE_PRECONDITION(args.size() > 0)
    for (auto arg : args) _args.push(arg);
}

String ArgumentStream::peek() const {
    ARIADNE_PRECONDITION(not empty())
    return _args.front();
}

String ArgumentStream::pop() {
    auto val = peek();
    _args.pop();
    return val;
}

Bool ArgumentStream::empty() const {
    return _args.empty();
}

SizeType ArgumentStream::size() const {
    return _args.size();
}

ArgumentPack::ArgumentPack(String const& id, VoidFunction const& processor) : _id(id), _processor(processor) { }

String const& ArgumentPack::id() const {
    return _id;
}

Void ArgumentPack::process() const {
    _processor();
}

Bool ArgumentPack::operator<(ArgumentPack const& other) const {
    return this->_id < other._id;
}

//! \brief Supplies base behavior for argument parsers
class ArgumentParserBase : public ArgumentParserInterface {
  protected:
    ArgumentParserBase(String const& short_id, String const& long_id, String const& instructions)
        : _short_id(short_id), _long_id(long_id), _instructions(instructions) { }
  public:
    String short_id() const { return _short_id; }
    String long_id() const { return _long_id; }
    Bool has_short_id() const { return not _short_id.empty(); }
    String instructions() const { return _instructions; }

    ArgumentPack consume(ArgumentStream& stream) const override;

    //! \brief If a value is required for this argument
    virtual Bool requires_value() const = 0;

    Bool is_consumable(ArgumentStream const& stream) const override;
    SizeType help_description_header_size() const override;
    String help_description(SizeType num_chars_to_align_instructions) const override;

    virtual ~ArgumentParserBase() = default;

  protected:
    virtual VoidFunction create_processor(ArgumentStream& stream) const = 0;

  private:
    String _short_id;
    String _long_id;
    String _instructions;
};

SizeType ArgumentParserBase::help_description_header_size() const {
    SizeType result = 4+long_id().size();
    if (has_short_id()) result+= 4+short_id().size();
    if (requires_value()) result += 8;
    return result;
}

String ArgumentParserBase::help_description(SizeType num_chars_to_separate_instructions) const {
    StringStream ss;
    ss << "[";
    if (has_short_id()) { ss << "-" << short_id() << " | "; }
    ss << "--" << long_id() << "]";
    if (requires_value()) { ss << " <value>"; }
    ss << std::string(num_chars_to_separate_instructions,' ') << instructions();
    return ss.str();
}

Bool ArgumentParserBase::is_consumable(const ArgumentStream &stream) const {
    auto arg = stream.peek();
    if (has_short_id()) {
        std::stringstream short_argument;
        short_argument << "-" << short_id();
        if (arg == short_argument.str()) return true;
    }
    String long_argument = "--"+long_id();
    return (arg == long_argument);
}

ArgumentPack ArgumentParserBase::consume(ArgumentStream& stream) const {
    stream.pop(); // Pop out the identifier already recognised as the one for this parser
    return ArgumentPack(_long_id,create_processor(stream));
}

class ValuedArgumentParserBase : public ArgumentParserBase {
  public:
    ValuedArgumentParserBase(String const& s, String const& l, String const& i) : ArgumentParserBase(s,l,i) { }
    Bool requires_value() const override { return true; }
    VoidFunction create_processor(ArgumentStream& stream) const override final {
        VoidFunction f;
        if (stream.empty()) { throw MissingArgumentValueException(long_id()); }
        try {
            f = _create_processor(stream);
        } catch (std::exception& e) {
            throw InvalidArgumentValueException(long_id());
        }
        return f;
    }
  protected:
    virtual VoidFunction _create_processor(ArgumentStream& stream) const = 0;
};

class UnvaluedArgumentParserBase : public ArgumentParserBase {
  public:
    UnvaluedArgumentParserBase(String const& s, String const& l, String const& i) : ArgumentParserBase(s,l,i) { }
    Bool requires_value() const override { return false; }
};

class ConcurrencyArgumentParser : public ValuedArgumentParserBase {
  public:
    ConcurrencyArgumentParser() : ValuedArgumentParserBase(
            "c","concurrency","Choose the concurrency as a non-negative integer <value> or use 'max' to choose the hardware concurrency of this machine (default: 0)") { }

    VoidFunction _create_processor(ArgumentStream& stream) const override {
        String arg = stream.pop();
        if (arg == "max") return []{ ThreadManager::instance().set_maximum_concurrency(); };
        else {
            int val = std::stoi(arg);
            if (val < 0) throw std::exception();
            return [val]{ ThreadManager::instance().set_concurrency(static_cast<unsigned int>(val)); };
        }
    }
};

class DrawerArgumentParser : public ValuedArgumentParserBase {
  public:
    DrawerArgumentParser() : ValuedArgumentParserBase(
            "d","drawer","Choose the drawer as a <value> in [ box | grid@X | affine@X | none ] with X a non-negative integer, where none disables drawing (default: affine@0)") { }

    VoidFunction _create_processor(ArgumentStream& stream) const override {
        String value = stream.pop();
        Drawer drawer = NullDrawer();
        if (value != "none") {
            if (value == "box") {
                drawer = BoxDrawer();
            } else {
                std::size_t at_pos = value.find('@',0);
                if (at_pos == std::string::npos) throw std::exception();
                else {
                    int accuracy = stoi(value.substr(at_pos+1));
                    if (accuracy < 0) throw std::exception();
                    String drawer_name = value.substr(0,at_pos);
                    if (drawer_name == "grid") drawer = GridDrawer((Nat)accuracy);
                    else if (drawer_name == "affine") drawer = AffineDrawer((Nat)accuracy);
                    else throw std::exception();
                }
            }
        }
        return [drawer]{ GraphicsManager::instance().set_drawer(drawer); };
    }
};

class GraphicsBackendArgumentParser : public ValuedArgumentParserBase {
  public:
    GraphicsBackendArgumentParser() : ValuedArgumentParserBase(
            "g","graphics","Choose the graphics backend as a <value> in [ cairo | gnuplot | none ], where none disables writing to file (default: cairo)") { }

    VoidFunction _create_processor(ArgumentStream& stream) const override {
        String val = stream.pop();
        #ifdef HAVE_CAIRO_H
	    if (val == "cairo") return []{ GraphicsManager::instance().set_backend(CairoGraphicsBackend()); };
        #endif
        #ifdef HAVE_GNUPLOT_H
	    if (val == "gnuplot") return []{ GraphicsManager::instance().set_backend(GnuplotGraphicsBackend()); };
        #endif
        if (val == "none") return []{ GraphicsManager::instance().set_backend(NullGraphicsBackend()); };
	    throw std::exception();
    }
};

class HelpArgumentParser : public UnvaluedArgumentParserBase {
  public:
    HelpArgumentParser() : UnvaluedArgumentParserBase(
            "h","help","Show this list of supported arguments") { }

    VoidFunction create_processor(ArgumentStream& stream) const override {
        return []{};
    }
};

class SchedulerArgumentParser : public ValuedArgumentParserBase {
public:
    SchedulerArgumentParser() : ValuedArgumentParserBase(
            "s","scheduler","Choose the logging scheduler as a <value> in [ immediate | blocking | nonblocking ] (default: nonblocking)") { }

    VoidFunction _create_processor(ArgumentStream& stream) const override {
        String val = stream.pop();
        if (val == "immediate") return []{ Logger::instance().use_immediate_scheduler(); };
        else if (val == "blocking") return []{ Logger::instance().use_blocking_scheduler(); };
        else if (val == "nonblocking") return []{ Logger::instance().use_nonblocking_scheduler(); };
        else throw std::exception();
    }
};

class ThemeArgumentParser : public ValuedArgumentParserBase {
  public:
    ThemeArgumentParser() : ValuedArgumentParserBase(
            "t","theme","Choose the logging theme as a <value> in [ none | light | dark ] (default: none)") { }

    VoidFunction _create_processor(ArgumentStream& stream) const override {
        TerminalTextTheme theme = TT_THEME_NONE;
        String val = stream.pop();
        if (val != "none") {
            if (val == "light") theme = TT_THEME_LIGHT;
            else if (val == "dark") theme = TT_THEME_DARK;
            else throw std::exception();
        }
        return [theme]{ Logger::instance().configuration().set_theme(theme); };
    }
};

class VerbosityArgumentParser : public ValuedArgumentParserBase {
  public:
    VerbosityArgumentParser() : ValuedArgumentParserBase(
            "v","verbosity","Choose the logging verbosity as a non-negative integer <value> (default: 0)") { }

    VoidFunction _create_processor(ArgumentStream& stream) const override {
        int val = std::stoi(stream.pop());
        if (val < 0) throw std::exception();
        return [val]{ Logger::instance().configuration().set_verbosity(static_cast<unsigned int>(val)); };
    }
};

CommandLineInterface::CommandLineInterface() : _parsers({
    ConcurrencyArgumentParser(),DrawerArgumentParser(),GraphicsBackendArgumentParser(),HelpArgumentParser(),
    SchedulerArgumentParser(),ThemeArgumentParser(),VerbosityArgumentParser()
    }) { }

Bool CommandLineInterface::acquire(int argc, const char* argv[]) const {
    List<String> args;
    for (int i = 0; i < argc; ++i) args.append(String(argv[i]));
    return acquire(args);
}

Bool CommandLineInterface::acquire(List<String> const& args) const {
    ArgumentStream stream(args);
    stream.pop(); // Pop out the function pointer or script name

    Set<ArgumentPack> packs;
    while (not stream.empty()) {
        Bool consumed = false;
        for (auto& parser : _parsers) {
            if (parser.is_consumable(stream)) {
                consumed = true;
                try {
                    auto p = parser.consume(stream);
                    if (packs.contains(p)) {
                        std::clog << "Argument '" << p.id() << "' specified multiple times.\n\n"
                            << parser.help_description(4) << std::endl;
                        return false;
                    } else if (p.id() == "help") {
                        p.process(); _print_help();
                        return false;
                    } else if (p.id() == "scheduler") {
                        // Need to process this before the others
                        p.process();
                    } else packs.insert(p);
                } catch(MissingArgumentValueException& exc) {
                    std::clog << "A value is required by the '" << exc.what() << "' argument, but it is not supplied.\n\n"
                        << parser.help_description(4) << std::endl;
                    return false;
                } catch(InvalidArgumentValueException& exc) {
                    std::clog << "Invalid '" << exc.what() << "' argument value, see the usage below:" << "\n\n" << parser.help_description(4) << std::endl;
                    return false;
                }
                break;
            }
        }
        if (not consumed) {
            std::clog << "Unrecognised command-line argument '" << stream.peek() << "'" << std::endl << std::endl;
            _print_help();
            return false;
        }
    }

    for (auto p : packs) p.process();

    return true;
}

void CommandLineInterface::_print_help() const {

    List<SizeType> chars(_parsers.size(),0);
    SizeType max_chars = 0;
    for (SizeType i=0; i<_parsers.size(); ++i)  {
        chars[i] = _parsers[i].help_description_header_size();
        if (chars[i] > max_chars) max_chars = chars[i];
    }

    std::clog << "Supported arguments:" << std::endl;
    for (SizeType i=0; i<_parsers.size(); ++i) {
        std::clog << "    " << _parsers[i].help_description(4+max_chars-chars[i]) << std::endl;
    }
}

} // namespace Ariadne
