// Own
#include "opensim_model.h"

// Project
#include <OpenSim/OpenSim.h>

using namespace SimTK;
using namespace OpenSim;

OpenSimModel::OpenSimModel()
{
	std::cout << "OpenSim constructor start." << std::endl;
	init();
	std::cout << "OpenSim constructor end." << std::endl;
}

void OpenSimModel::init()
{
	//*
	Model model;
	model.setName("example");

	// Create two links, each with a mass of 1 kg, center of mass at the body's
	// origin, and moments and products of inertia of zero.
	OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body* radius = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

	// Connect the bodies with pin joints. Assume each body is 1 m long.
	PinJoint* shoulder = new PinJoint("shoulder",
		// Parent body, location in parent, orientation in parent.
		model.getGround(), Vec3(0), Vec3(0),
		// Child body, location in child, orientation in child.
		*humerus, Vec3(0, 1, 0), Vec3(0));
	PinJoint* elbow = new PinJoint("elbow",
		*humerus, Vec3(0), Vec3(0), *radius, Vec3(0, 1, 0), Vec3(0));

	// Add a muscle that flexes the elbow.
	Millard2012EquilibriumMuscle* biceps = new
		Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
	biceps->addNewPathPoint("origin", *humerus, Vec3(0, 0.8, 0));
	biceps->addNewPathPoint("insertion", *radius, Vec3(0, 0.7, 0));

	// Add a controller that specifies the excitation of the muscle.
	PrescribedController* brain = new PrescribedController();
	brain->addActuator(*biceps);
	// Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
	brain->prescribeControlForActuator("biceps",
		new StepFunction(0.5, 3, 0.3, 1));

	// Add components to the model.
	model.addBody(humerus);    model.addBody(radius);
	model.addJoint(shoulder);  model.addJoint(elbow);
	model.addForce(biceps);
	model.addController(brain);

	// Add a console reporter to print the muscle fiber force and elbow angle.
	ConsoleReporter* reporter = new ConsoleReporter();
	reporter->set_report_time_interval(1.0);
	reporter->addToReport(biceps->getOutput("fiber_force"));
	reporter->addToReport(
		elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
		"elbow_angle");
	model.addComponent(reporter);
	//*/
}