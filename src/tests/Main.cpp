#include <iostream>

#include "kinematics/Skeleton.h"
#include "kinematics/FileInfoSkel.hpp"

#include "utils/Paths.h"
#include "manipulability/ManipulabilityPaths.h"
#include "utils/UtilsMath.h"
#include "simulation/World.h"

using namespace rtql8::kinematics;
using namespace rtql8::dynamics;
using namespace rtql8::simulation;
using namespace rtql8::utils;

int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
	FileInfoSkel<Skeleton> model;
    model.loadFile(RTQL8MANIPULABILITY_DATA_PATH"/skeletons/fullbody1.skel", SKEL);
    model.loadFile(RTQL8_DATA_PATH"/skel/Chain.skel", SKEL);
    SkeletonDynamics *skel = (SkeletonDynamics*)model.getSkel();
	std::cout << "no errors found \n";
}
