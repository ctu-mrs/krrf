/*
 * File name: gnuplot.h
 * Date:      2010/06/27
 * Author:    Miroslav Kulich
 */

#ifndef IMR_GNUPLOT
#define IMR_GNUPLOT

#include <stdio.h>
#include <string>


namespace CPainters {

class Gnuplot {

  public:
    static const int endl;
    Gnuplot();
    ~Gnuplot();

    void showonscreen();
    void cmd(const std::string &cmdstr);

  private:
    FILE *gnucmd;
    
};

}

#endif

