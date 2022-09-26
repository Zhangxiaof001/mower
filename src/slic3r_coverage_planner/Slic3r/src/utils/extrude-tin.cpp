#include "Config.hpp"
#include "Model.hpp"
#include "IO.hpp"
#include "TriangleMesh.hpp"
#include "libslic3r.h"
#include <boost/nowide/args.hpp>
#include <boost/nowide/iostream.hpp>

using namespace Slic3r;

void confess_at(const char *file, int line, const char *func, const char *pat, ...){}

int
main(int argc, char **argv)
{
    // Convert arguments to UTF-8 (needed on Windows).
    // argv then points to memory owned by a.
    boost::nowide::args a(argc, argv);
    
    // read config
    ConfigDef config_def;
    {
        ConfigOptionDef* def;
        
        def = config_def.add("offset", coFloat);
        def->label = "Offset from the lowest point (min thickness)";
        def->cli = "offset";
        def->default_value = new ConfigOptionFloat(1);
    
        def = config_def.add("output", coString);
        def->label = "Output File";
        def->tooltip = "The file where the output will be written (if not specified, it will be based on the input file).";
        def->cli = "output";
        def->default_value = new ConfigOptionString("");
    }
    DynamicConfig config(&config_def);
    t_config_option_keys input_files;
    config.read_cli(argc, argv, &input_files);
    
    for (t_config_option_keys::const_iterator it = input_files.begin(); it != input_files.end(); ++it) {
        TriangleMesh mesh;
        Slic3r::IO::STL::read(*it, &mesh);
        mesh.extrude_tin(config.option("offset", true)->getFloat());
        
        std::string outfile = config.option("output", true)->getString();
        if (outfile.empty()) outfile = *it + "_extruded.stl";
        
        Slic3r::IO::STL::write(mesh, outfile);
        boost::nowide::cout << "Extruded mesh written to " << outfile << std::endl;
    }
    
    return 0;
}
