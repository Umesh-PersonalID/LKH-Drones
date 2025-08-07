# cplex.pri - Update paths for your system
# Based on installation path: ~/IBM/ILOG/CPLEX_Studio_Community2212

# Use $$PWD or absolute paths. $$PWD refers to the directory containing this .pri file.
# For absolute path, replace with /home/umesh/IBM/ILOG/CPLEX_Studio_Community2212

CPLEX_ROOT = /home/umesh/IBM/ILOG/CPLEX_Studio_Community2212

INCLUDEPATH += $$CPLEX_ROOT/concert/include
INCLUDEPATH += $$CPLEX_ROOT/cplex/include

# Add -lilocplex HERE, before the other CPLEX libs
LIBS += -L$$CPLEX_ROOT/cplex/lib/x86-64_linux/static_pic -lilocplex -lcplex
LIBS += -L$$CPLEX_ROOT/concert/lib/x86-64_linux/static_pic -lconcert
LIBS += -ldl

# If you encounter linking issues related to C++ ABI or standard library,
# you might need to add the CPLEX C++ library (cxx) as well:
# LIBS += -L$$CPLEX_ROOT/cplex/lib/x86-64_linux/static_pic -lcplex -lcplexcxx -lilocplex
# And potentially its include path if separate:
# INCLUDEPATH += $$CPLEX_ROOT/cplex/include/ilcplex

# Ensure the paths above match the actual structure inside your CPLEX installation directory.