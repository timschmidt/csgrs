/*

iRRAM_STREAMS.h -- interface to the C++-style IO for the iRRAM library
 
Copyright (C) 2004-2009 Norbert Mueller
 
This file is part of the iRRAM Library.
 
The iRRAM Library is free software; you can redistribute it and/or modify
it under the terms of the GNU Library General Public License as published by
the Free Software Foundation; either version 2 of the License, or (at your
option) any later version.
 
The iRRAM Library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library General Public
License for more details.
 
You should have received a copy of the GNU Library General Public License
along with the iRRAM Library; see the file COPYING.LIB.  If not, write to
the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
MA 02111-1307, USA. 
*/

#ifndef iRRAM_STREAMS_H
#define iRRAM_STREAMS_H

#include <iostream>
#include <istream>
#include <ostream>
#include <iomanip>
#include <string>

#include <iRRAM/core.h>

namespace iRRAM {

struct _SetRwidth { int _M_n; };

inline _SetRwidth  setRwidth(int __n) {
  _SetRwidth __x;
  __x._M_n = __n;
  return __x;
}

struct _SetRflags { float_form _M_n; };

inline _SetRflags  setRflags(float_form n) {
  _SetRflags x;
  x._M_n = n;
  return x;
}

class rstream {};

class orstream :public rstream
{
public:
  bool fail();
  bool eof();
  bool bad();
  bool good(); 
  operator bool(); 
  friend  bool operator !(orstream&);
  unsigned int real_w;
  float_form real_f;

  orstream(std::string s, std::ios::openmode mod=std::ios::out);
  ~orstream();

  bool is_open();
  void open(std::string s, std::ios::openmode mod=std::ios::out);
  void close();

  orstream& operator<<(const bool b);
  orstream& operator<<(const char* ch);
  orstream& operator<<(const short i);
  orstream& operator<<(const unsigned short i);
  orstream& operator<<(const int i);
  orstream& operator<<(const unsigned int i);
  orstream& operator<<(const long i);
  orstream& operator<<(const unsigned long i);
  orstream& operator<<(const long long i);
  orstream& operator<<(const unsigned long long i);
  orstream& operator<<(const double d);
  orstream& operator<<(const float d);
  orstream& operator<<(const std::string& s);
  orstream& operator<<(const REAL& r);
  orstream& operator<<(const RATIONAL& r);
  orstream& operator<<(const INTEGER& r);
  orstream& operator<<(const DYADIC& d);

  orstream& operator<<(decltype(std::setw(0)));
  orstream& operator<<(decltype(std::setprecision(0)));
  orstream& operator<<(decltype(std::setfill('*')));
  orstream& operator<<(decltype(std::setiosflags(std::ios_base::dec)));
  orstream& operator<<(decltype(std::resetiosflags(std::ios_base::dec)));
  orstream& operator<<(decltype(std::setbase(0)));

  orstream& operator<<(_SetRwidth);
  orstream& operator<<(_SetRflags);
  orstream& operator<<(std::ostream&(std::ostream&) );
#if iRRAM_HAVE_TLS
  orstream& operator<<(const std::thread::id &);
#endif

// the following should be "private:"

  orstream();
  // Default constructor, only for "rout": Will select cout as target.

  orstream(std::ostream *target, bool respect_iteration);
  // Internal constructor, only for "cout", "cerr", and "clog".

  void rewind();
  // this will reset the requests counter.
  void reset();
  // this will reset both counters.

  std::ostream *target;
  bool _respect_iteration;



};

class irstream :public rstream
{
public:
  bool fail();
  bool eof();
  bool bad();
  bool good(); 
  operator bool(); 
  friend  bool operator !(irstream&);

  irstream(std::string s, std::ios::openmode mod=std::ios::in);
  ~irstream();

  bool is_open();
  void open(std::string s, std::ios::openmode mod=std::ios::in);
  void close();

  irstream& operator>>(bool& i);
  irstream& operator>>(short& i);
  irstream& operator>>(unsigned short& i);
  irstream& operator>>(int& i);
  irstream& operator>>(unsigned int& i);
  irstream& operator>>(long& i);
  irstream& operator>>(unsigned long& i);
  irstream& operator>>(double& d);
  irstream& operator>>(float& d);
  irstream& operator>>(long long& d);
  irstream& operator>>(unsigned long long& d);
  irstream& operator>>(std::string& s);
  irstream& operator>>(REAL& d);
  irstream& operator>>(INTEGER& d);
  irstream& operator>>(DYADIC& d);

  /* these are not implemented: */
  irstream& operator>>(decltype(std::setw(0)));
  irstream& operator>>(decltype(std::setprecision(0)));
  irstream& operator>>(decltype(std::setfill('*')));
  irstream& operator>>(decltype(std::setiosflags(std::ios_base::dec)));
  irstream& operator>>(decltype(std::resetiosflags(std::ios_base::dec)));
  irstream& operator>>(decltype(std::setbase(0)));

// the following should be "private:"
  irstream();

private:
  std::istream *target;

};

extern iRRAM_TLS irstream cin;
extern iRRAM_TLS orstream cout;
extern iRRAM_TLS orstream cerr;
extern iRRAM_TLS orstream clog;

}

#endif
