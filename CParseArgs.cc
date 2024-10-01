#include "CParseArgs.h"
#include <stdlib.h>

template<>
void Option<double>::parseVal(char *v) {
    *value = atof(v);
    used++;
}

template<>
void Option<int>::parseVal(char *v) {
    *value = atoi(v);
    used++;
}

template<>
void Option<char *>::parseVal(char *v) {
    *value = v;
    used++;
}


template<>
void CmdOptions::addOption(const TOptionInt &o) {
    ios.push_back(o);
	cmdOrder.push_back(std::pair<int,int>(0,ios.size()-1));
}

template<>
void CmdOptions::addOption(const TOptionDouble &o) {
    dos.push_back(o);
	cmdOrder.push_back(std::pair<int,int>(1,dos.size()-1));
}

template<>
void CmdOptions::addOption(const TOptionChar &o) {
    cos.push_back(o);
	cmdOrder.push_back(std::pair<int,int>(2,cos.size()-1));
}



