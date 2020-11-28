#ifndef SEARCH_H
#define SEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <vector>
#include <unordered_map>
#include <math.h>
#include <limits>
#include <list>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <chrono>

class Search
{
    public:
        Search();
        virtual ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

    protected:
        std::unordered_map<int, Node> OPEN, CLOSED;

        //Get Heuristic function for several cases
        virtual double getHeuristic(int a1, int b1, const Map& map, const EnvironmentOptions& options);
        virtual double getHeuristic(int x1, int y1, int x2, int y2, const EnvironmentOptions& options);
        virtual Node argmin(const EnvironmentOptions& options);
        virtual std::list<Node> getSuccessors(Node s, const Map& map, const EnvironmentOptions& options);
        virtual Node changeParent(Node currentNode, Node parentNode, const Map& map, const EnvironmentOptions& options);
        virtual bool lineOfSight(int x1, int y1, int x2, int y2, const Map& map, const EnvironmentOptions& options);
        virtual void makePrimaryPath(Node currentNode);
        virtual void makeSecondaryPath();
        SearchResult sresult; //This will store the search result
        std::list<Node> lppath, hppath; //
        
        //CODE HERE to define other members of the class
};
#endif
