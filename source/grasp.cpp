#include <franka/gripper.h>
#include <string.h>
#include <iostream>

int _main(int argc, char **argv)
{
	try
	{
		if (argc == 2)
        {
            franka::Gripper gripper(argv[1]);
            gripper.move(0.08, 0.01);
        }
        else if (argc == 3)
        {
            franka::Gripper gripper(argv[1]);
            char *end;
            double force = strtod(argv[2], &end);
            if (*end != '\0') throw std::runtime_error("Invalid usage. Usage: IP [force]");
            gripper.grasp(0.0, 0.01, force, 0.1, 0.1);
        }
        else throw std::runtime_error("Invalid usage. Usage: IP [force]");
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