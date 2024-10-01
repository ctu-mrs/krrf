#ifndef _CPARSE_ARGS_H_
#define _CPARSE_ARGS_H_

#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

/*
template<typename T>
struct is_pointer { static const bool value = false; };

template<typename T>
struct is_pointer<T*> { static const bool value = true; };
*/

template<typename T>
struct Option {
	std::string name, description, sname;
    T *value;
    bool mandatory;
    int used;
	T defaultvalue;
    Option(const std::string &n, T *ptr, const std::string &desc):
        name(n), description(desc), value(ptr), mandatory(true),used(0), defaultvalue(0) {
			setShortName();
        }
	Option(const std::string &n, T *ptr, T defvalue, const std::string &desc):
		name(n), description(desc), value(ptr), mandatory(false),used(0),defaultvalue(defvalue) {
			setShortName();
		}


    Option(const Option<T> &rhs):
        name(rhs.name), description(rhs.description), sname(rhs.sname), value(rhs.value), mandatory(rhs.mandatory),
		   used(rhs.used), defaultvalue(rhs.defaultvalue) {}



    std::string toString(const int width, const bool printNonZero) const {
		// for char * type these is a specialized funcion
		std::stringstream ss,dvs;
        std::stringstream delim;
        const int nw = width - name.size();
        for(int i=0;i<nw;i++) {
            delim << " ";
        }
		
		if (!mandatory) {
			if (printNonZero==false && defaultvalue == NULL) {
				dvs << "[NULL]";
			} else {
				dvs << "[" << defaultvalue << "] ";
			}
		}	    
		ss << name << delim.str() << dvs.str() << description;
        return ss.str();
    }

    void parseVal(char *v);
	void setShortName() {
		char ss[20];
		snprintf(ss,sizeof(ss),"-%s",name.c_str());
		sname = std::string(ss);
	}

	void setDefault(const bool printNonZero) {
		*value = defaultvalue;
		used++;
        if (printNonZero) {
            std::cerr << "Option [" << name << "] set to default value=" << defaultvalue << "\n";
        } else {
            if (defaultvalue != NULL) {
                std::cerr << "Option [" << name << "] set to default value=" << defaultvalue << "\n";
            } else {
                std::cerr << "Option [" << name << "] set to default value=NULL\n";
            }
        }
	}

	std::string getCmd() const {
		std::stringstream ss;
		ss << ( mandatory ? "" : "[");
		ss << sname;
		ss << ( mandatory ? "" : "]");
		return ss.str();
	}
};

/*
template<>
inline std::string Option<char *>::toString(const int width) const {
	// for char * = we should check, if defaultval is not null
	std::stringstream ss;
	std::stringstream delim;
	const int nw = width - name.size();
	for(int i=0;i<nw;i++) {
		delim << " ";
	}
	if (defaultvalue == NULL) {
		ss << name << delim.str() << "[NULL] " << description;
	} else {
		ss << name << delim.str() << "[" << *defaultvalue << "] " << description;
	}
	return ss.str();

}
*/



class CmdOptions {
    public:
    typedef Option<int> TOptionInt;
    typedef Option<double> TOptionDouble;
    typedef Option<char *> TOptionChar;

	std::vector< TOptionInt > ios;
	std::vector< TOptionDouble > dos;
	std::vector< TOptionChar > cos;

	// 0 == ios, 1=dos, 2=cos;
	std::vector< std::pair<int, int> > cmdOrder;

    template<typename T>
    void addOption(const T &o);

    std::string printHelp() const {
		std::stringstream ss;
        const int w = getNameLengthMax() + 4;

		for(int i=0;i<(int)cmdOrder.size();i++) {
			if (cmdOrder[i].first == 0) {
				ss << ios[cmdOrder[i].second].toString(w,true) << "\n";
			} else if (cmdOrder[i].first == 1) {
				ss << dos[cmdOrder[i].second].toString(w,true) << "\n";
			} else {
				ss << cos[cmdOrder[i].second].toString(w,true) << "\n";
			}
		}
        return ss.str();
    }

	template<typename T>
    std::string getcmd(const std::vector<Option<T> > &options) const {
		std::stringstream ss;
		for(int i=0;i<(int)options.size();i++) {
			ss << options[i].getCmd() << " ";
		}
		return ss.str();
	}


    template<typename T>
    int getNameLengthMax(const std::vector<Option<T> > &options) const {
        int m=-1;
        for(int i=0;i<(int)options.size();i++) {
            m = std::max<int>(m,options[i].name.size());
        }
        return m;
    }


    int getNameLengthMax() const {
        int m = std::max( std::max( getNameLengthMax(ios), getNameLengthMax(dos) ) , getNameLengthMax(cos) );
        return m;
    }

	std::string makeCmdLine() const {
		std::stringstream ss;
		for(int i=0;i<(int)cmdOrder.size();i++) {
			if (cmdOrder[i].first == 0) {
				ss << ios[cmdOrder[i].second].getCmd() << " ";
			} else if (cmdOrder[i].first == 1) {
				ss << dos[cmdOrder[i].second].getCmd() << " ";
			} else {
				ss << cos[cmdOrder[i].second].getCmd() << " ";
			}
		}
		return ss.str();
	}

    
    template<typename T>
    bool parse(const int argc, char **argv, std::vector<Option<T> > &options) {
        for(int i = 0; i <(int)options.size();i++) {
            for(int j=0;j<argc;j++) {
                if (strcmp(argv[j],options[i].sname.c_str()) == 0) {
                    j++;
                    if (j < argc) {
                        options[i].parseVal(argv[j]);
                    } else {
                        std::cerr << "Cannot parse value of option '" << options[i].name << "', because argc=" << argc << "\n";
                        return false;                
                    }
                }
            }
        }
        return true;
    }

    template<typename T>
    bool checkMandatory(const std::vector<Option<T> > &options) {
        bool ok = true;
        for(int i=0;i<(int)options.size();i++) {
            if (options[i].mandatory == true && options[i].used==0) {
				std::cerr << "Option '" << options[i].name << "' was not set!\n";
                ok = false;
            }
        }
        return ok;
    }

	template<typename T>
	void assignDefaults(std::vector<Option<T> > &options, const bool printNonZero) {
		for(int i=0;i<(int)options.size();i++) {
			if (options[i].mandatory == false && options[i].used == 0) {
				options[i].setDefault(printNonZero);
			}
		}
	}


    bool parse(const int argc, char **argv) {
        bool p1 = parse(argc,argv,ios);
        bool p2 = parse(argc,argv,dos);
        bool p3 = parse(argc,argv,cos);
        if (!p1 || !p2 || ! p3) {
            return false;
        }

		assignDefaults(ios, true);
		assignDefaults(dos ,true);
		assignDefaults(cos, false);

        bool b1 = checkMandatory(ios);
        bool b2 = checkMandatory(dos);
        bool b3 = checkMandatory(cos);
        return b1 && b2 && b3;
    }



};



#endif


