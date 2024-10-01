
#include "gnuplot.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <sstream>              // for std::ostringstream
#include <stdexcept>
#include <cstdio>
#include <cstdlib>              // for getenv()


namespace CPainters {

    const int Gnuplot::endl = -999999;


    Gnuplot::Gnuplot() {
        gnucmd = popen("gnuplot","w");

    }

    Gnuplot::~Gnuplot() {
        pclose(gnucmd);
    }

    void Gnuplot::showonscreen() {
        cmd(std::string("set output\nset terminal x11\n"));
    }

    void Gnuplot::cmd(const std::string &cmdstr) {
        fputs( (cmdstr).c_str(), gnucmd );
        fflush(gnucmd);
    }

} // namespace CPainters


