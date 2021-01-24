#include "environmentoptions.h"

EnvironmentOptions::EnvironmentOptions() {
    searchtype = CN_SP_ST_DIJK;
    metrictype = CN_SP_MT_EUCL;
    allowsqueeze = false;
    allowdiagonal = true;
    cutcorners = false;
    breakingties = CN_SP_BT_GMAX;
    hweight = 1;
}

EnvironmentOptions::EnvironmentOptions(bool AS, bool AD, bool CC, int MT, double HW, int BT) {
    metrictype = MT;
    allowsqueeze = AS;
    allowdiagonal = AD;
    cutcorners = CC;
    hweight = HW;
    breakingties = BT;
}