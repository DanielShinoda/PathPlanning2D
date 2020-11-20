#include "search.h"

#include <algorithm>
#include <chrono>

Search::Search()
{
//set defaults here
}

Search::~Search() {}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start, finish;
    start = std::chrono::system_clock::now();

    Node currNode;
    // Initialize start point
    currNode.i = map.getStartI();
    currNode.j = map.getStartJ();
    currNode.g = 0.0;
    currNode.H = computeHeuristic(currNode.i, currNode.j, map.getGoalI(), map.getGoalJ(), options);
    Open.insert({currNode.i + (currNode.j * map.getMapHeight()), currNode});

    bool pathFound = false;
    
    while (!Open.empty()) {
        currNode = findMin(options);

        Close.insert({ currNode.i + (currNode.j * map.getMapHeight()), currNode });
        Open.erase(currNode.i + (currNode.j * map.getMapHeight()));

        if (currNode.i == map.getGoalI() && currNode.j == map.getGoalJ()) {
            pathFound = true;
            break;
        }

        auto successors = getNeighbours(currNode, map, options);

        for (auto iter = successors.begin(); iter != successors.end(); ++iter) {
            if ((Open.find(iter->i + (iter->j * map.getMapHeight())) == Open.end()) ||
                ((iter->F < Open[iter->i + (iter->j * map.getMapHeight())].F)
                    || ((iter->F == Open[iter->i + (iter->j * map.getMapHeight())].F)
                        && (((options.breakingties) && (iter->g >= Open[iter->i + (iter->j * map.getMapHeight())].g))
                            || ((!(options.breakingties)) && (iter->g <= Open[iter->i + (iter->j * map.getMapHeight())].g)))))) {
                iter->parent = &(Close.find(currNode.i + (currNode.j * map.getMapHeight()))->second);
                *iter = changeParent(*iter, *(iter->parent), map, options);
                Open.erase(iter->i + (iter->j * map.getMapHeight()));
                Open.insert({ iter->i + (iter->j * map.getMapHeight()) }, *iter);
            }
        }
        Logger->writeToLogOpenClose(Open, Close, false);
    }

    Logger->writeToLogOpenClose(Open, Close, true);

    if (pathFound) {
        sresult.pathlength = (float)currNode.g;
        makePrimaryPath(currNode);
    }

    finish = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count()) / 1000000000;

    if (pathFound) {
        makeSecondaryPath();
    }


    sresult.pathfound = pathFound;
    sresult.nodescreated = (unsigned int)(Close.size() + Open.size());
    sresult.numberofsteps = (unsigned int)Close.size();
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

double Search::computeHeuristic(int a1, int b1, int a2, int b2, const EnvironmentOptions& options)
{
    double H = 0.0;

    if (options.searchtype < 2) {
        return 0.0;
    }

    if (options.metrictype == CN_SP_MT_DIAG) {
        H = (std::min(abs(a1 - a2), abs(b1 - b2)) * sqrt(2)) + abs((abs(a1 - a2) - abs(b1 - b2)));
    }

    if (options.metrictype == CN_SP_MT_MANH) {
        H = abs(a1 - a2) + abs(b1 - b2);
    }

    if (options.metrictype == CN_SP_MT_EUCL) {
        H = sqrt((a1 - a2) * (a1 - a2) + (b1 - b2) * (b1 - b2));
    }

    if (options.metrictype == CN_SP_MT_CHEB) {
        H = std::max(abs(a1 - a2), abs(b1 - b2));
    }

    return H;
}

bool Search::lineOfSight(int x1, int y1, int x2, int y2, const Map& map, const EnvironmentOptions& options) {
    int dx, dy, x_inc, y_inc, error, error0, n, x, y;

    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    x_inc = (x2 > x1 ? 1 : -1);
    y_inc = (y2 > y1 ? 1 : -1);
    n = dx + dy;
    error = dx - dy;
    error0 = error;
    x = x1;
    y = y1;
    dx *= 2;
    dy *= 2;

    if (dx == 0) {
        for (; y != y2; y += y_inc) {
            if (map.CellIsObstacle(x, y))
                return false;
        }
        return true;
    }
    else if (dy == 0) {
        for (; x != x2; x += x_inc) {
            if (map.CellIsObstacle(x, y))
                return false;
        }
        return true;
    }

    while (n > 0) {
        if (map.CellIsObstacle(x, y)) {
            return false;
        }

        if (error > 0) {
            if ((!options.cutcorners) && ((error - dy) < error0)) {
                if (map.CellIsObstacle(x, y + y_inc)) {
                    return false;
                }
            }
            x += x_inc;
            error -= dy;

        }
        else if (error < 0) {
            if ((!options.cutcorners) && ((error + dx) > error0)) {
                if (map.CellIsObstacle(x + x_inc, y)) {
                    return false;
                }
            }
            y += y_inc;
            error += dx;

        }
        else if (error == 0) {
            if (map.CellIsTraversable(x + x_inc, y)) {
                x += x_inc;
                error -= dy;
            }
            else if (map.CellIsTraversable(x, y + y_inc)) {
                y += y_inc;
                error += dx;
            }
            else {
                return false;
            }
        }
        --n;
    }

    if (map.CellIsObstacle(x, y)) {
        return false;
    }

    return true;
}

std::list<Node> Search::getNeighbours(Node currentNode, const Map& map, const EnvironmentOptions& options)
{
    std::list<Node> neighbors;
    Node neighbor;
    bool noWay;

    for (int down = -1; down < 2; ++down) {
        for (int right = -1; right < 2; ++right) {
            noWay = false;

            if ((down != 0) || (right != 0)) {

                if (map.CellOnGrid(currentNode.i + down, currentNode.j + right) &&
                    map.CellIsTraversable(currentNode.i + down, currentNode.j + right)) {
                    if ((down != 0) && (right != 0)) {
                        if ((map.CellIsObstacle(currentNode.i + down, currentNode.j)) &&
                            (map.CellIsObstacle(currentNode.i, currentNode.j + right)) &&
                            (!(options.allowsqueeze))) {
                            noWay = true;
                        }

                        if (!(options.allowdiagonal)) {
                            noWay = true;
                        }

                        if (((map.CellIsObstacle(currentNode.i + down, currentNode.j)) ||
                            (map.CellIsObstacle(currentNode.i, currentNode.j + right))) &&
                            (!(options.cutcorners))) {
                            noWay = true;
                        }
                    }

                    if ((!(noWay)) &&
                        (Close.find((currentNode.i + down) + (map.getMapHeight() * (currentNode.j + right))) == Close.end())) {
                        neighbor.i = currentNode.i + down;
                        neighbor.j = currentNode.j + right;
                        if ((down != 0) && (right != 0)) {
                            neighbor.g = currentNode.g + sqrt(2);
                        }
                        else {
                            neighbor.g = currentNode.g + 1.0;
                        }
                        neighbor.H = computeHeuristic(neighbor.i, neighbor.j, map.getGoalI(), map.getGoalJ(), options);
                        neighbor.F = neighbor.g + (options.hweight * neighbor.H);

                        neighbors.push_front(neighbor);
                    }
                }
            }
        }
    }
    return neighbors;
}

Node Search::findMin(const EnvironmentOptions& options)
{
    Node minNode;
    minNode = (Open.begin())->second;
    for (auto it = Open.begin(); it != Open.end(); ++it) {
        if (it->second.F < minNode.F) {
            minNode = it->second;
        }
        else if (it->second.F == minNode.F) {
            if (options.breakingties) {
                if (it->second.g >= minNode.g) {
                    minNode = it->second;
                }
            }
            else {
                if (it->second.g <= minNode.g) {
                    minNode = it->second;
                }
            }
        }
    }
    return minNode;
}

Node Search::changeParent(Node currentNode, Node parentNode, const Map& map, const EnvironmentOptions& options) {
    if (options.searchtype != CN_SP_ST_TH) {
        return currentNode;
    }
    if ((parentNode.parent == nullptr) || ((currentNode.i == parentNode.parent->i) && (currentNode.j == parentNode.parent->j))) {
        return currentNode;
    }
    if (lineOfSight(parentNode.parent->i, parentNode.parent->j, currentNode.i, currentNode.j, map, options)) {
        currentNode.g = parentNode.parent->g +
            sqrt(((currentNode.i - parentNode.parent->i) * (currentNode.i - parentNode.parent->i)) +
            ((currentNode.j - parentNode.parent->j) * (currentNode.j - parentNode.parent->j)));
        currentNode.F = currentNode.g + (options.hweight * currentNode.H);
        currentNode.parent = parentNode.parent;
    }
    return currentNode;
}

void Search::makePrimaryPath(Node currNode)
{
    Node thisNode = currNode;

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
