#ifndef _WLOG_H__
#define _WLOG_H__

//#define NOWLOG 

#include <iostream>
#include <sstream>
#include <fstream>
#include <ostream>

class _WLog {

	public:
		_WLog(const char *filename, std::ostream &os = std::cerr);
		~_WLog();
		void flush();

		std::stringstream _logStream;
		std::ofstream _ofs;
		std::ostream &_os;

};

extern _WLog *__wlog;

#define VARNAME(x) #x 
#define WVAR(x) VARNAME(x) << "=" << x 

#ifndef NOWLOG
	#define WDEBUG(x) if (__wlog != NULL) { __wlog->_logStream << x; __wlog->flush(); }
#else
	#define WDEBUG(x) ;
#endif

#ifndef NOWLOG
	#define WWDEBUG(x) if (__wlog != NULL) { __wlog->_logStream << __PRETTY_FUNCTION__ << ":" << __LINE__ << " D " << x; __wlog->flush(); }
#else
	#define WWDEBUG(x) ;
#endif




#ifndef NOWLOG
	#define WERROR(x) if (__wlog != NULL) { __wlog->_logStream << " E " << x; __wlog->flush(); }
#else
	#define WERROR(x) ;
#endif

#ifndef NOWLOG
	#define WWARNING(x) if (__wlog != NULL) { __wlog->_logStream << " W " << x; __wlog->flush(); }
#else
	#define WWARNING(x) ;
#endif


/**
#define WOUTPUT_DEFAULT "\033[0m"
#define WOUTPUT_BLACK "\033[30m"
#define WOUTPUT_RED "\033[31m"
#define WOUTPUT_GREEN "\033[32m"
#define WOUTPUT_YELLOW "\033[33m"
#define WOUTPUT_BLUE "\033[34m"
#define WOUTPUT_MAGENTA "\033[35m"
#define WOUTPUT_CYAN "\033[36m"
#define WOUTPUT_WHITE "\033[37m"




#ifndef DISABLE_LOG
     #define WMAGENTA(x) WINFO( WOUTPUT_MAGENTA <<  x << WOUTPUT_DEFAULT )
#else
     #define WMAGENTA(x) ;
#endif

#ifndef DISABLE_LOG
     #define WCYAN(x) WINFO( WOUTPUT_CYAN <<  x << WOUTPUT_DEFAULT )
#else
     #define WCYAN(x) ;
#endif

#ifndef DISABLE_LOG
     #define WGREEN(x) WINFO( WOUTPUT_GREEN <<  x << WOUTPUT_DEFAULT )
#else
     #define WGREEN(x) ;
#endif

#ifndef DISABLE_LOG
     #define WWHITE(x) WINFO( WOUTPUT_WHITE <<  x << WOUTPUT_DEFAULT )
#else
     #define WWHITE(x) ;
#endif

#ifndef DISABLE_LOG
     #define WBLUE(x) WINFO( WOUTPUT_BLUE <<  x << WOUTPUT_DEFAULT )
#else
     #define WBLUE(x) ;
#endif

#ifndef DISABLE_LOG
     #define WBLACK(x) WINFO( WOUTPUT_BLACK <<  x << WOUTPUT_DEFAULT )
#else
     #define WBLACK(x) ;
#endif

**/



#endif


