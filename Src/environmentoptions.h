#ifndef ENVIRONMENTOPTIONS_H
#define ENVIRONMENTOPTIONS_H
#include "gl_const.h"

class EnvironmentOptions {
public:
    EnvironmentOptions(bool AS, bool AD, bool CC, int MT, double HW, int BT);
    EnvironmentOptions();
    int searchtype;
    int     metrictype;     //Can be chosen Euclidean, Manhattan, Chebyshev and Diagonal distance
    bool    allowsqueeze;   //Option that allows to move throught "bottleneck"
    bool    allowdiagonal;  //Option that allows to make diagonal moves
    bool    cutcorners;     //Option that allows to make diagonal moves, when one adjacent cell is untraversable
    bool    breakingties;
    double  hweight;

};

#endif // ENVIRONMENTOPTIONS_H
