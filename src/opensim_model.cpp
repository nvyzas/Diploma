// Own
#include "opensim_model.h"

// Project
#include <OpenSim/OpenSim.h> // causes runtime error

using namespace OpenSim;

OpenSimModel::OpenSimModel()
{
	Model osimModel;
	osimModel.setName("tugOfWar");
	std::cout << "OpenSim example completed successfully.\n";
}