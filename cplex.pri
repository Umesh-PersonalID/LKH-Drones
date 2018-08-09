DEFINES += IL_STD

CPXROOT = /opt/ibm/ILOG/CPLEX_Studio128

LIBS += -lconcert -lilocplex -lcplex \
    -L$${CPXROOT}/concert/lib/x86-64_linux/static_pic \
    -L$${CPXROOT}/cplex/lib/x86-64_linux/static_pic \

INCLUDEPATH += $${CPXROOT}/concert/include \
    $${CPXROOT}/cplex/include \

DEPENDPATH += $${CPXROOT}/concert/include \
    $${CPXROOT}/cplex/include \
