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
#include <chrono>

class Search
{
    public:
        Search();
        virtual ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

    protected:
        //Hint 1. You definetely need class variables for OPEN and CLOSE
        std::unordered_map<int, Node> Open, Close;

        //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
        //for non-heuristic search methods like Dijkstra

        //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
        //will return it's sucessors, e.g. unordered list of nodes

        //Hint 4. working with OPEN and CLOSE is the core
        //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
        //Start with very simple (and ineffective) structures like list or vector and make it work first
        //and only then begin enhancement!
        virtual Node findMin(const EnvironmentOptions& options);
        virtual double computeHeuristic(int a1, int b1, int a2, int b2, const EnvironmentOptions& options);
        virtual std::list<Node> getNeighbours(Node currentNode, const Map& map, const EnvironmentOptions& options);

        virtual void makePrimaryPath(Node currentNode);
        virtual void makeSecondaryPath();

        virtual bool lineOfSight(int x1, int y1, int x2, int y2, const Map& map, const EnvironmentOptions& options);
        virtual Node changeParent(Node currentNode, Node parentNode, const Map& map, const EnvironmentOptions& options);


        SearchResult                    sresult; //This will store the search result
        std::list<Node>                 lppath, hppath; //
        
        //CODE HERE to define other members of the class
};
#endif
