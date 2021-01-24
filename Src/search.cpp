#include "search.h"
#include <chrono>

Search::Search()
{
    //set defaults here
}

Search::~Search() {}


SearchResult Search::startSearch(ILogger* Logger, const Map& map, const EnvironmentOptions& options)
{
    auto time = std::chrono::steady_clock::now();

    Node s;
    s.i = map.getStartI();
    s.j = map.getStartJ();
    s.g = 0;
    s.parent = nullptr;
    s.H = getHeuristic(s.i, s.j, map, options);
    s.F = s.g + (options.hweight * s.H);
    bool pathFound(0);
    // It is made to avoid same hashing for (i, j), (j, i) nodes
    OPEN.insert({ s.j + s.i * map.getMapWidth(), s });
    while (!OPEN.empty()) {
        s = argmin(options);
        CLOSED.insert({ s.j + s.i * map.getMapWidth(), s });
        OPEN.erase(s.j + s.i * map.getMapWidth());
        // Check if current node is our "finish" node
        if (s.i == map.getGoalI() && s.j == map.getGoalJ()) {
            pathFound = 1;
            break;
        }
        std::list<Node> successors = getSuccessors(s, map, options);
        for (auto& iter : successors) {
            if (check(iter, map, options)) {
                iter.parent = &(CLOSED.find(s.i * map.getMapWidth() + s.j)->second);
                OPEN.erase(iter.i * map.getMapWidth() + iter.j);
                OPEN.insert({ iter.i * map.getMapWidth() + iter.j , iter });
            }
        }
        Logger->writeToLogOpenClose(OPEN, CLOSED, 0);
    }
    Logger->writeToLogOpenClose(OPEN, CLOSED, 1);
    sresult.pathfound = pathFound;
    sresult.nodescreated = CLOSED.size() + OPEN.size();
    sresult.numberofsteps = CLOSED.size();
    if (pathFound) {
        sresult.pathlength = s.g;
        makePrimaryPath(s);
    }
    sresult.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - time).count();
    if (pathFound) makeSecondaryPath();
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

// returns true if current node needs to be updated in OPEN hashmap
bool Search::check(const Node& current, const Map& map, const EnvironmentOptions& options) {
    // if current node not in OPEN
    if (OPEN.find(current.i * map.getMapWidth() + current.j) == OPEN.end()) return true;
    // if current F-value is smaller than old one
    if (current.F < OPEN[current.i * map.getMapWidth() + current.j].F) return true;
    // if F-values equal then compare g-values depending on breakingties option
    if (current.F == OPEN[current.i * map.getMapWidth() + current.j].F) {

        if ((options.breakingties) && (current.g >= OPEN[current.i * map.getMapWidth() + current.j].g)) return true;

        if (!(options.breakingties) && (current.g <= OPEN[current.i * map.getMapWidth() + current.j].g)) return true;
    }

    return false;
}

double Search::getHeuristic(int x1, int y1, const Map& map, const EnvironmentOptions& options) {
    if (options.searchtype < 2) return 0.0;

    int x2(map.getGoalI()), y2(map.getGoalJ());
    double dx(abs(x1 - x2)), dy(abs(y1 - y2));

    if (options.metrictype == CN_SP_MT_DIAG) return (std::min(dx, dy) * CN_SQRT_TWO) + abs(dx - dy);

    if (options.metrictype == CN_SP_MT_MANH) return (dx + dy);

    if (options.metrictype == CN_SP_MT_EUCL) return (sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));

    if (options.metrictype == CN_SP_MT_CHEB) return std::max(dx, dy);
}

//Finds min cost node in OPEN
Node Search::argmin(const EnvironmentOptions& options) {
    Node minNode = (OPEN.begin())->second;
    for (auto iter = OPEN.begin(); iter != OPEN.end(); ++iter) {
        Node cur = iter->second;
        if (cur.F < minNode.F) {
            minNode = cur;
        } else if (cur.F == minNode.F) {
            if (options.breakingties) {
                if (cur.g >= minNode.g) minNode = cur;
            } else {
                if (cur.g <= minNode.g) minNode = cur;
            }
        }
    }
    return minNode;
}

std::list<Node> Search::getSuccessors(const Node& s, const Map& map, const EnvironmentOptions& options) {
    std::list<Node> successors;
    Node successor;
    bool isMove;

    for (int horizontal = -1; horizontal < 2; ++horizontal) {
        for (int vertical = -1; vertical < 2; ++vertical) {
            isMove = false;

            if ((horizontal != 0) || (vertical != 0)) {

                if (map.CellIsCorrect(s.i + horizontal, s.j + vertical)) {
                    if ((horizontal != 0) && (vertical != 0)) {
                        if ((map.CellIsObstacle(s.i + horizontal, s.j)) &&
                            (map.CellIsObstacle(s.i, s.j + vertical)) &&
                            (!(options.allowsqueeze))) isMove = true;

                        if (!(options.allowdiagonal)) isMove = true;

                        if (((map.CellIsObstacle(s.i + horizontal, s.j)) ||
                            (map.CellIsObstacle(s.i, s.j + vertical))) &&
                            (!(options.cutcorners))) isMove = true;
                    }

                    if ((!(isMove)) &&
                        (CLOSED.find((s.i + horizontal) * map.getMapWidth() + (s.j + vertical)) == CLOSED.end())) {
                        successor.i = s.i + horizontal;
                        successor.j = s.j + vertical;
                        if ((horizontal != 0) && (vertical != 0)) successor.g = s.g + CN_SQRT_TWO;
                        else successor.g = s.g + 1.0;
                        successor.H = getHeuristic(successor.i, successor.j, map, options);
                        successor.F = successor.g + (options.hweight * successor.H);
                        successors.push_front(successor);
                    }
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
