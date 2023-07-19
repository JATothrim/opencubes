#include <iostream>

#include "cmdparser.hpp"
#include "config.hpp"
#include "cubes.hpp"

void configure_arguments(cli::Parser& parser) {
    parser.set_required<int>("n", "cube_size", "the size of polycube to generate up to");
    parser.set_optional<int>("t", "threads", 1, "the number of threads to use while generating");
    parser.set_optional<bool>("c", "use_cache", false, "whether to load cache files");
    parser.set_optional<bool>("w", "write_cache", false, "wheather to save cache files");
    parser.set_optional<bool>("v", "version", false, "print build version info");
}

int main(int argc, char** argv) {
    cli::Parser parser(argc, argv);
    configure_arguments(parser);
    parser.run_and_exit_if_error();

    if (parser.get<bool>("v")) {
        std::printf("Built with %s, %s, %s CUBES_MAX_N:%d\n", CONFIG_VERSION, CONFIG_BUILDTYPE, CONFIG_COMPILERID, CUBES_MAX_N);
    }
    gen(parser.get<int>("n"), parser.get<int>("t"), parser.get<bool>("c"), parser.get<bool>("w"));
    return 0;
}
