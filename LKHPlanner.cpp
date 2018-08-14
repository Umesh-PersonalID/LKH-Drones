//comments by JLB
#include "LKHPlanner.h"

#include <QDebug>

extern "C" {
#include "linkern.h"
#include "util.h"
#include "kdtree.h"

typedef struct distobj {
    CCdatagroup *dat;
    int       *cacheval;
    int       *cacheind;
    int        cacheM;
} distobj;

double cycle_length(int ncount, int *cyc, distobj *D);
}

QVector<QVector<quint32> > g_dist;
QVector<QVector<QVector<quint32> > > g_turn;

void setData(int ncount, CCdatagroup *D, quint32 res, quint32 pcs, quint32 lambda, quint32 gamma, quint32 kappa) {
    // sqrt(2) < 1.5 < 2
    qreal max_dist = 1.5 * res;//upper bound on distance
    g_dist = QVector<QVector<quint32>>(ncount);//init vector of vector of ints of capcity equalling the total number of nodes (container needs to hold 1 onto 1 relation)
    for (int i = 0; i < ncount; i++) {//for each index (i) less than the total node count
        g_dist[i] = QVector<quint32>(ncount);//set up a vector for each node to contain distance to the other nodes
        for (int j = 0; j < ncount; j++) {//for each index (j) less than the total node count
            qreal dist = QGeoCoordinate(D->x[i], D->y[i]).distanceTo(QGeoCoordinate(D->x[j], D->y[j]));//calculate the distance from node i to node j 
            if (!dist || dist > max_dist) {//first condition means o distance, second means the calculated distance is farther than possible
                g_dist[i][j] = kappa;//TODO: not sure why we use kappa, which is defined as the drones carrying capacity
            } else {
                g_dist[i][j] = lambda * pcs * dist;//TODO: I think pcs has to do with size of the grid, confirm
            }
        }
    }

    g_turn = QVector<QVector<QVector<quint32> > >(ncount);//set vector of vector of vector of ints of capcity equalling the total number of nodes
    for (int i = 0; i < ncount; i++) {//for each index (i) less than the total node count
        g_turn[i] = QVector<QVector<quint32> >(ncount);
        for (int j = 0; j < ncount; j++) {//for each index (j) less than the total node count
            g_turn[i][j] = QVector<quint32>(ncount);
            for (int k = 0; k < ncount; k++) {//for each index (k) less than the total node count
                if (g_dist[i][j] > lambda * pcs * max_dist || g_dist[j][k] > lambda * pcs * max_dist) {
                    g_turn[i][j][k] = kappa;//TODO: not sure why we use kappa, which is defined as the drones carrying capacity
                } else {
                    qreal r = QGeoCoordinate(D->x[i], D->y[i]).distanceTo(QGeoCoordinate(D->x[j], D->y[j]));//1st side of the triangle
                    qreal e = QGeoCoordinate(D->x[j], D->y[j]).distanceTo(QGeoCoordinate(D->x[k], D->y[k]));//2nd side of the triangle
                    qreal s = QGeoCoordinate(D->x[k], D->y[k]).distanceTo(QGeoCoordinate(D->x[i], D->y[i]));//3rd side of the triangle
                    qreal t = (r * r + e * e - s * s) / (2.0 * r * e);
                    if (t > 1.0) {
                        t = 1.0;
                    } else if (t < -1.0) {
                        t = -1.0;
                    }

                    qreal turn = M_PI - acos(t);//calculate angle

                    g_turn[i][j][k] = gamma; //JLB this used to also be here when angle mattered->* pcs * turn;//removing this sets the constant gamma to be a constant factor for all turns
                }
            }
        }
    }
}

void clearData() {
    for (int i = 0; i < g_dist.size(); i++) {
        g_dist[i].clear();
    }
    g_dist.clear();

    for (int i = 0; i < g_dist.size(); i++) {
        for (int j = 0; j < g_dist.size(); j++) {
            g_turn[i][j].clear();
        }
        g_turn[i].clear();
    }
    g_turn.clear();
}

double cycle_length(int ncount, int *cyc, distobj *D) {
    quint32 dist = 0;
    for (int i = 0; i < ncount; i++) {
        dist += g_dist[cyc[i]][cyc[(i + 1) % ncount]];
    }

    quint32 turn = 0;
    for (int i = 1; i < ncount; i++) {
        turn += g_turn[cyc[i - 1]][cyc[i]][cyc[(i + 1) % ncount]];
    }

    return dist + turn;
}

LKHPlanner::LKHPlanner(UBPlanner *parent) : UBPlanner(parent),
    m_rnd(1000)
{
}

bool LKHPlanner::planAgent(quint32 agent) {
    bool result = false;

    int ncount = m_agent_paths[agent].size(), seed = 0;
    double val, kzeit;
    double startzeit;
    int tempcount, *templist;
    int *incycle = (int *) NULL, *outcycle = (int *) NULL;
    CCdatagroup dat;
    CCrandstate rstate;

    seed = (int) CCutil_real_zeit ();
    CCutil_sprand (seed, &rstate);

    printf ("Chained Lin-Kernighan with seed %d\n", seed);
    fflush (stdout);

    startzeit = CCutil_zeit ();

    CCutil_init_datagroup (&dat);
    CCutil_dat_setnorm (&dat, CC_GEOM);

    dat.x = CC_SAFE_MALLOC (ncount, double);
    dat.y = CC_SAFE_MALLOC (ncount, double);

    for (int i = 0; i < m_agent_paths[agent].size(); i++) {
        dat.x[i] = m_nodes[m_agent_paths[agent][i].first].latitude();
        dat.y[i] = m_nodes[m_agent_paths[agent][i].first].longitude();
    }

    setData(ncount, &dat, m_res, m_pcn, m_lambda, m_gamma, m_kappa);

    incycle = CC_SAFE_MALLOC (ncount, int);
    outcycle = CC_SAFE_MALLOC (ncount, int);

    CCkdtree localkt;

    kzeit = CCutil_zeit ();
    if (CCkdtree_build (&localkt, ncount, &dat, (double *) NULL,
                        &rstate)) {
        fprintf (stderr, "CCkdtree_build failed\n");
        goto CLEANUP;
    }
    printf ("Time to build kdtree: %.2f\n", CCutil_zeit () - kzeit);
    fflush (stdout);

    kzeit = CCutil_zeit ();
    if (CCkdtree_quadrant_k_nearest (&localkt, ncount, 2,
           &dat, (double *) NULL, 1, &tempcount, &templist,
           0, &rstate)) {
        fprintf (stderr, "CCkdtree-quad nearest code failed\n");
        goto CLEANUP;
    }
    printf ("Time to find quad %d-nearest: %.2f\n",
            2, CCutil_zeit () - kzeit);
    fflush (stdout);

    kzeit = CCutil_zeit ();
    if (CCkdtree_qboruvka_tour (&localkt, ncount,
              &dat, incycle, &val, &rstate)) {
        fprintf (stderr, "CCkdtree qboruvka-tour failed\n");
        goto CLEANUP;
    }
    printf ("Time to grow tour: %.2f\n",
            CCutil_zeit () - kzeit);
    fflush (stdout);

    CCkdtree_free (&localkt);

    kzeit = CCutil_zeit ();
    if (CClinkern_tour (ncount, &dat, tempcount, templist, 10000000,
           m_rnd, incycle, outcycle, &val, 0,
           -1, -1, NULL, CC_LK_WALK_KICK,
           &rstate)) {
        fprintf (stderr, "CClinkern_tour failed\n");
        goto CLEANUP;
    }
    printf ("Lin-Kernighan Running Time: %.2f\n",
            CCutil_zeit () - kzeit);

    printf ("Final Cycle: %.0f\n", val);
    fflush (stdout);

    printf ("Total Running Time: %.2f\n", CCutil_zeit () - startzeit);
    fflush (stdout);

    for (int i = 0; i < ncount; i++) {
        m_agent_paths[agent][outcycle[i]].second = m_agent_paths[agent][outcycle[(i + 1) % ncount]].first;
    }

    result = true;

CLEANUP:

#ifndef BIG_PROBLEM
    CC_IFFREE (templist, int);
#endif

    CC_IFFREE (dat.x, double);
    CC_IFFREE (dat.y, double);

    CC_IFFREE (incycle, int);
    CC_IFFREE (outcycle, int);

    CCutil_freedatagroup (&dat);

    clearData();

    return result;
}
