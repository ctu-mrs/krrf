#ifndef CSTAT_H_
#define CSTAT_H_

#include <map>
#include <iostream>
#include <string>
#include <sstream>
#include "ut.h"
//#include "hdf5io.h"

namespace rrtPlanning {

class CStat {
	public:

	typedef std::map<std::string, double>::iterator iterator;
	typedef std::map<std::string, double>::const_iterator const_iterator;
	
    typedef std::map<std::string, std::string>::iterator siterator;
	typedef std::map<std::string, std::string>::const_iterator sconst_iterator;

	CStat(){
	}

	CStat(const CStat &rhs):values(rhs.values), valuesString( rhs.valuesString) {}



	~CStat() {
		values.clear();
        valuesString.clear();
	}

/*
	std::string print(const std::string &delimiter="\n", const std::string &last="\n") const {
		stringstream ss;
		int j = 0;
		for(const_iterator i = values.begin(); i != values.end();i++) {
			ss << "\'" << (*i).first << "\'" << " = " << (*i).second;
			if (j < (int)values.size()-1) {
				ss << delimiter;
			}
			j++;
		}
		ss << last;
		return ss.str();
	}
*/
    std::string printFull() const {
        stringstream ss;
        for(const_iterator i = values.begin(); i != values.end();i++) {
            ss << (*i).first << "=" << (*i).second << ";";
        }
        for(sconst_iterator i = valuesString.begin(); i != valuesString.end();i++) {
            const std::string &value((*i).second);
            if (value != "") {
                ss << (*i).first << "=" << (*i).second << ";";
            } else {
                ss << (*i).first << "='" << (*i).second << "';";
            }
        }
        return ss.str();
    }


	void clear() {
		values.clear();
        valuesString.clear();
	}

	void zero() {
		for(iterator i = values.begin(); i != values.end();i++) {
			values[(*i).first] = 0;
		}
		for(siterator i = valuesString.begin(); i != valuesString.end();i++) {
			valuesString[(*i).first] = "";
		}
	}

	double &operator[](const std::string &idx) {
	    return values[idx];
	}

    std::string getStrValue(const std::string &idx) {
        return valuesString[ idx ];
    }

    void setStrValue(const std::string &idx, const std::string &value) {
        valuesString[idx] = value;
    }

	CStat &operator=(const CStat &rhs) {
		if (this != &rhs) {
			values = rhs.values;
            valuesString = rhs.valuesString;
		}
		return *this;
	}

    CStat &operator+=(const CStat &rhs) {
		if (this != &rhs) {
            for(const_iterator i = rhs.values.begin(); i != rhs.values.end(); i++) {
                values[(*i).first] += (*i).second;
            }
		}
		return *this;
	}


    const std::map<std::string,double> &getValues() const { return values; }
    const std::map<std::string,std::string> &getValuesString() const { return valuesString; }

	private:
	std::map<std::string,double> values;
	std::map<std::string,std::string> valuesString;
};

#undef DOOWNSTAT

} // namespace rrtPlanning

#endif

