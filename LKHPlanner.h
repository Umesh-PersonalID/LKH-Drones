#ifndef LKHPLANNER_H
#define LKHPLANNER_H

#include "UBPlanner.h"

#include <QObject>

class LKHPlanner : public UBPlanner
{
    Q_OBJECT
public:
    explicit LKHPlanner(UBPlanner *parent = 0);

protected:
    quint32 m_rnd;

protected:
    virtual bool planAgent(quint32 agent);

public:
    void setRound(quint32 rnd) {m_rnd = rnd;}
};

#endif // LKHPLANNER_H
