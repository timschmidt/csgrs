find_path(KIRK_INCLUDE_DIR kirk/kirk-real-obj.h PATH_SUFFIXES kirk)
#check_include_files(kirk/kirk-real-obj.h HAVE_KIRK_REAL_OBJ_H)
find_library(KIRK_LIBRARY kirk)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Kirk DEFAULT_MSG KIRK_LIBRARY KIRK_INCLUDE_DIR)

#Diagnostic messages
#message(KIRK_FOUND=${KIRK_FOUND})
#message(KIRK_LIBRARY=${KIRK_LIBRARY})
#message(KIRK_INCLUDE_DIR=${KIRK_INCLUDE_DIR})
#message(HAVE_KIRK_REAL_OBJ_H=${HAVE_KIRK_REAL_OBJ_H})

