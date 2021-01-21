#include "search.h"
#include <chrono>


Search::Search()
{
    //set defaults here
}

Search::~Search() {}


SearchResult Search::startSearch(ILogger* Logger, const Map& map, const EnvironmentOptions& options)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    Node s;
    s.i = map.getStartI();
    s.j = map.getStartJ();
    s.g = 0;
    s.parent = nullptr;
    s.H = getHeuristic(s.i, s.j, map, options);
    s.F = s.g + s.H;
    bool pathFound(false);

    // It is made to avoid same hashing for (i, j), (j, i) nodes
    OPEN.insert({ s.j + s.i * map.getMapWidth(), s });
    while (!OPEN.empty()) {
        s = argmin(options);
        OPEN.erase(s.j + s.i * map.getMapWidth());
        CLOSED.insert({ s.j + s.i * map.getMapWidth(), s });
        //Check if current node is our "finish" node
        if (s.i == map.getGoalI() && s.j == map.getGoalJ()) {
            //We found the path to the finish node
            pathFound = true;
            break;
        }

        std::list<Node> successors = getSuccessors(s, map, options);
        //Here are some cases to not choose the node
        //1) Node not in OPEN
        //2) Node cost is inappropriate
        for (auto iter = successors.begin(); iter != successors.end(); ++iter) {
            if ((OPEN.find(iter->i * map.getMapWidth() + iter->j) == OPEN.end()) ||
                ((iter->F < OPEN[iter->i * map.getMapWidth() + iter->j].F) ||
                ((iter->F == OPEN[iter->i * map.getMapWidth() + iter->j].F) &&
                (((options.breakingties) && (iter->g >= OPEN[iter->i * map.getMapWidth() + iter->j].g)) ||
                ((!(options.breakingties)) && (iter->g <= OPEN[iter->i * map.getMapWidth() + iter->j].g)))))) {

                iter->parent = &(CLOSED.find(s.i * map.getMapWidth() + s.j)->second);
                OPEN.erase(iter->i * map.getMapWidth() + iter->j);
                OPEN.insert({ iter->i * map.getMapWidth() + iter->j , *iter });
            }
        }

        Logger->writeToLogOpenClose(OPEN, CLOSED, false);

    }
    Logger->writeToLogOpenClose(OPEN, CLOSED, true);

    if (pathFound) {
        sresult.pathlength = s.g;
        makePrimaryPath(s);
    }


    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTime - startTime;
    sresult.time = duration.count();

    if (pathFound) makeSecondaryPath();

    sresult.pathfound = pathFound;
    sresult.nodescreated = CLOSED.size() + OPEN.size();
    sresult.numberofsteps = CLOSED.size();

    sresult.hppath = &hppath;
    sresult.lppath = &lppath;

    return sresult;
}

double Search::getHeuristic(int x1, int y1, int x2, int y2, const EnvironmentOptions& options) {
    double dx(abs(x1 - x2)), dy(abs(y1 - y2));

    if (options.searchtype < 2) { return 0.0; }

    if (options.metrictype == CN_SP_MT_DIAG) { return (std::min(dx, dy) * sqrt(2)) + abs(dx - dy); }

    if (options.metrictype == CN_SP_MT_MANH) { return (dx + dy); }

    if (options.metrictype == CN_SP_MT_EUCL) { return (sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))); }

    if (options.metrictype == CN_SP_MT_CHEB) { return std::max(dx, dy); }
}

double Search::getHeuristic(int x1, int y1, const Map& map, const EnvironmentOptions& options) {
    int x2(map.getGoalI()), y2(map.getGoalJ());
    double dx(abs(x1 - x2)), dy(abs(y1 - y2));

    if (options.searchtype < 2) { return 0.0; }

    if (options.metrictype == CN_SP_MT_DIAG) { return (std::min(dx, dy) * sqrt(2)) + abs(dx - dy); }

    if (options.metrictype == CN_SP_MT_MANH) { return (dx + dy); }

    if (options.metrictype == CN_SP_MT_EUCL) { return (sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))); }

    if (options.metrictype == CN_SP_MT_CHEB) { return std::max(dx, dy); }
}

//Finds min cost node in OPEN
Node Search::argmin(const EnvironmentOptions& options) {
    Node minNode = (OPEN.begin())->second;
    for (auto iter = OPEN.begin(); iter != OPEN.end(); ++iter) {
        Node cur = iter->second;
        if (cur.F < minNode.F) {
            minNode = cur;
        } else if (cur.F == minNode.F) {
            if (cur.g <= minNode.g) {
                minNode = cur;
            }
        }
    }
    return minNode;
}

std::list<Node> Search::getSuccessors(Node s, const Map& map, const EnvironmentOptions& options) {
    std::list<Node> successors;
    Node successor;
    bool noWay;
    for (int vertical = -1; vertical < 2; ++vertical) {
        for (int horizontal = -1; horizontal < 2; ++horizontal) {
            noWay = false;

            if (map.CellIsCorrect(s.i + vertical, s.j + horizontal)) {

                if ((vertical != 0) && (horizontal != 0)) {

                    if (options.allowdiagonal) {
                        bool diag1 = map.CellIsCorrect(s.i, s.j + horizontal), diag2 = map.CellIsCorrect(s.i + vertical, s.j);
                        if ((options.cutcorners && ((!diag1 && !diag2 && options.allowsqueeze) ||
                            (diag1 && !diag2) || (!diag1 && diag2) || (diag1 && diag2)))) {
                            noWay = false;
                        } else {
                            noWay = true;
                        }
                    }
                }
                //If there is a way and node not in CLOSED
                if ((!noWay) && (CLOSED.find((s.i + vertical) * map.getMapWidth() + (s.j + horizontal)) == CLOSED.end())) {
                    successor.i = s.i + vertical;
                    successor.j = s.j + horizontal;
                    if ((vertical != 0) && (horizontal != 0)) successor.g = s.g + sqrt(2);
                    else successor.g = s.g + 1;
                    successor.H = getHeuristic(successor.i, successor.j, map, options);
                    successor.F = successor.g + successor.H * options.hweight;
                    successors.push_front(successor);
                }
            }
            }
        }
    return successors;
}

void Search::makePrimaryPath(Node currentNode) {
    Node thisNode = currentNode;

    while (thisNode.parent) {
        lppath.push_front(thisNode);
        thisNode = *(thisNode.parent);
    }
    lppath.push_front(thisNode);
}

void Search::makeSecondaryPath()
{
    auto it = lppath.begin();
    int currentNode_i, currentNode_j, nextNode_i, nextNode_j;
    hppath.push_back(*it);

    while (it != --lppath.end()) {
        currentNode_i = it->i;
        currentNode_j = it->j;
        ++it;
        nextNode_i = it->i;
        nextNode_j = it->j;
        ++it;
        if (((it->i - nextNode_i) != (nextNode_i - currentNode_i)) || ((it->j - nextNode_j) != (nextNode_j - currentNode_j))) {
            hppath.push_back(*(--it));
        }
        else {
            --it;
        }
    }
}
