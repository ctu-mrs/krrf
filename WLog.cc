
#include "WLog.h"
#include <time.h>
#include <stdio.h>
#include "gitv.h"

using namespace std;

//#define WLOG_REPORT_TIME 1

_WLog *__wlog = NULL;

_WLog::_WLog(const char *filename, std::ostream &os):_ofs(filename),_os(os)
{
	if (!_ofs.is_open()) {
		_os << "ERROR: cannot open " << filename << "for logging!\n";
	} else {
//		_logStream << " * start log";
        _logStream << "GIT " << GIT_VERSION << "\n";
		flush();
	}
}

_WLog::~_WLog() {
	_ofs.close();
}

void _WLog::flush() {

	//time_t tclock;
	//time(&tclock);

#ifdef WLOG_REPORT_TIME
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME,&ts);

	//struct tm *t = gmtime(&tclock);
	struct tm *t = gmtime(&(ts.tv_sec));

    char str[20];
    snprintf(str, sizeof(str), "%02d:%02d:%02d.%09d", t->tm_hour, t->tm_min, t->tm_sec, ts.tv_nsec);
		
    _ofs << str << " " ;
//	_ofs << t->tm_hour << ':' << t->tm_min << ':' << t->tm_sec << '.' << ts.tv_nsec;
#endif

	_ofs << _logStream.str() << std::endl << std::flush;
	_os << _logStream.str() << std::endl;

	_logStream.str(std::string(""));

}


