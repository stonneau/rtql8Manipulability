#include <iostream>

#include "kinematics/Skeleton.h"
#include "kinematics/FileInfoSkel.hpp"
#include "simulation/World.h"

#include "rtql8Manipulability\DecomposedSkeleton.h"

#include "utils/Paths.h"
#include "manipulability/ManipulabilityPaths.h"
#include "utils/UtilsMath.h"


using namespace rtql8::kinematics;
using namespace rtql8::dynamics;
using namespace rtql8::simulation;
using namespace rtql8::utils;

int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
	FileInfoSkel<Skeleton> model;
    model.loadFile(RTQL8MANIPULABILITY_DATA_PATH"/skeletons/fullbody1.skel", SKEL);
    Skeleton *skel = (Skeleton*)model.getSkel();
	DecomposedSkeleton decomposedSkel(skel);
	std::cout << "number of limbs" << decomposedSkel.limbs_.size();

	BodyNode* root =  skel->getRoot();
}
