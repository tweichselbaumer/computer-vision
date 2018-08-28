#include "Settings.h"

Settings::Settings(string filename)
{
	filename_ = filename;
}

void Settings::load()
{
	if (!boost::filesystem::exists(filename_))
	{
		/*std::ifstream i(filename_);
		json j;
		i >> j;*/
	}
	else 
	{

	}
}

void Settings::save()
{
	/*std::ofstream o(filename_);
	o << j << std::endl;*/
}