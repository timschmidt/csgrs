# Try to find the Ariadne library for rigorous numerics.
#
# Once done this will define:
#
#  ARIADNE_FOUND - system has Ariadne
#  ARIADNE_INCLUDE_DIRS - the Ariadne include directories
#  ARIADNE_LIBRARIES - Link these to use Ariadne

# This file is part of Ariadne.

# Ariadne is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Ariadne is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.

find_library(ARIADNE_LIBRARY ariadne)
set(ARIADNE_LIBRARIES "${ARIADNE_LIBRARY}")

find_path(ARIADNE_INCLUDE_DIR ariadne.hpp PATH_SUFFIXES ariadne)
get_filename_component(ARIADNE_INCLUDE_PARENT_DIR ${ARIADNE_INCLUDE_DIR} DIRECTORY)
set(ARIADNE_INCLUDE_DIRS ${ARIADNE_INCLUDE_PARENT_DIR} ${ARIADNE_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ariadne DEFAULT_MSG ARIADNE_LIBRARIES ARIADNE_INCLUDE_DIRS)

mark_as_advanced(
  ARIADNE_INCLUDE_DIRS
  ARIADNE_LIBRARIES
)
