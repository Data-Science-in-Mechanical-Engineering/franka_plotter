#include "../include/franka_plotter/plotter.h"
#include <stdexcept>
#include <iostream>

int _main(int argc, char **argv)
{
	try
	{
		if (argc != 2) throw std::runtime_error("Invalid usage");
		Drawing drawing(argv[1]);
		Plotter plotter;
		plotter.plot(drawing);
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