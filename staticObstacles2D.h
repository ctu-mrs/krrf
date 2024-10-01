#ifndef _STATIC_OBSTACLES_H_
#define _STATIC_OBSTACLES_H_

#include "ut.h"
#include "types.h"

namespace rrtPlanning {


std::vector< Triangle > addGap(const double leftX, const double rightX, const double npSize, const double funnelAngle, const double funnelDepth, const SDimension &mapDimension);


} // namespace


#endif



