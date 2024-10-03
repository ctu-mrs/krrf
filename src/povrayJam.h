#ifndef POVRAY_JAM_H
#define POVRAY_JAM_H

#include <vector>
#include <list>
#include <map>
#include <string>
#include <sstream>

namespace rrtPlanning {

class PJ {

    public:
    PJ();
    PJ(const char *prefix);

    ~PJ();


    char _cameraName[2000];
    char _robotName[2000];
    char _mapName[2000];



};

class PJParam {
    PJParam(const double v = 0) {
        _value = v;
        _valueDefined = true;
        _name = ""
    }

    PJParam(const char *value) {
        _valueDefined = false;
        _value = 0;
        _name = std::string(value);
        
    }
    std::string _name;
    bool _valueDefined;
    double _value;
};


}

#endif
