#include "../include/franka_plotter/configuration.h"
#include <franka/gripper.h>
#include <string.h>
#include <iostream>

int _main(int argc, char **argv)
{
	try
	{
		if (argc == 1)
        {
            Configuration configuration;
            franka::Gripper gripper(configuration.ip());
            gripper.move(0.08, 0.01);
        }
        else if (argc == 2)
        {
            char *end;
            double force = strtod(argv[1], &end);
            if (*end != '\0') throw std::runtime_error("Invalid usage. Usage: ./franka_plotter_grasp [force]");
            Configuration configuration;
            franka::Gripper gripper(configuration.ip());
            gripper.grasp(0.0, 0.01, force, 0.1, 0.1);
        }
        else throw std::runtime_error("Invalid usage. Usage: ./franka_plotter_grasp [force]");
	}
	catch (std::exception &e)
	{
		std::cerr << "Exception occured: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	return _main(argc, argv);
}