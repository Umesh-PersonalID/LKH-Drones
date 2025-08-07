#ifndef UBPLANNER_H
#define UBPLANNER_H

#include <QPair>
#include <QVector>
#include <QPolygonF>
#include <QGeoCoordinate>

#include <QObject>

class Waypoint;

class UBPlanner : public QObject
{
    Q_OBJECT
public:
    explicit UBPlanner(QObject *parent = 0);
    ~UBPlanner();

private:
signals:
    void planReady(); // emitter

public slots:
    void startPlanner();

protected:
    QString m_file;
    qreal m_res;
    quint32 m_limit;
    qreal m_gap;
    qreal m_lambda;
    qreal m_gamma;
    qreal m_gamma_45;
    qreal m_gamma_90;
    qreal m_gamma_135;
    quint32 m_kappa;

    quint32 m_pcn;

    QVector<quint32> m_depots;  // A vector used to map agents it's starting depot
    QVector<QPolygonF> m_areas; // A vector of defined areas, QPolygonF is a vector of points (QPointF) using floating point precision used to define this area

    QVector<QGeoCoordinate> m_nodes;  // vector of all nodes
    QVector<QGeoCoordinate> m_agents; // vector of all agents (start?) locations

    QVector<QVector<QPair<quint32, quint32>>> m_agent_paths; // VOV containing

    // binary obstacle map (300x300 grid: -1 for free space, 1 for obstacles)
    QVector<QVector<int>> m_obstacle_map;
    int m_grid_width;
    int m_grid_height;

protected:
    bool divide(); // divides the area amung the agents
    bool decompose();
    bool validatePath(quint32 agent);
    bool buildMission(quint32 agent);
    bool evaluate(const QVector<QPointF> &cell);
    void build_grid();

    QList<Waypoint *> loadWaypoints(const QString &loadFile);
    bool storeWaypoints(const QString &storeFile, QList<Waypoint *> &wps);

    virtual bool planAgent(quint32 agent);

    bool checkLineIntersection(const QGeoCoordinate &p, const QGeoCoordinate &q, const QVector<QVector<int>> &obstacle_map, int grid_width, int grid_height);

public:
    void setFile(const QString &file) { m_file = file; }
    void setResolution(qreal res) { m_res = res; }
    void setLimit(quint32 limit) { m_limit = limit; }
    void setGap(qreal gap) { m_gap = gap; }
    void setLambda(qreal lambda) { m_lambda = lambda; }
    void setGamma(qreal gamma) { m_gamma = gamma; }
    void setkappa(quint32 kappa) { m_kappa = kappa; }
    void setPrecision(quint32 pcn) { m_pcn = pcn; }
    bool loadObstacleMap(const QVector<QVector<int>> &grid_data, int width, int height);
    bool loadObstacleMapFromFile(const QString &filename, int width = 300, int height = 300);
};

#endif // UBPLANNER_H