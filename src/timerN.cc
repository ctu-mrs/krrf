/*
 * File name: timerN.cc
 * Date:      2005/10/31 08:33
 * Author:    Jan Faigl
 */

#include <cstdlib>
#include <sys/resource.h>
#include <unistd.h>

#include "timerN.h"

using namespace crl;

/// ----------------------------------------------------------------------------
/// Class CTimerN
/// ----------------------------------------------------------------------------
long CTimerN::processStartSec = 0;
long CTimerN::processStartUSec = 0;

/// ----------------------------------------------------------------------------
CTimerN::CTimerN()
{
   userStart_ = 0;
   sysStart_ = 0;
   realStart_ = 0;
   enabled_ = false;
   reset();
}

/// ----------------------------------------------------------------------------
CTimerN::CTimerN(const CTimerN &timer)
    : userStart_(timer.userStart_), sysStart_(timer.sysStart_),
      realStart_(timer.realStart_), rS(timer.rS), rE(timer.rE),
      userElapsed_(timer.userElapsed_), sysElapsed_(timer.sysElapsed_),
      realElapsed_(timer.realElapsed_), enabled_(timer.enabled_)
{
}

/// ----------------------------------------------------------------------------
CTimerN &CTimerN::operator=(const CTimerN &timer)
{
   if (this != &timer) {
      userStart_ = timer.userStart_;
      sysStart_ = timer.sysStart_;
      realStart_ = timer.realStart_;
      rS = timer.rS;
      rE = timer.rE;
      userElapsed_ = timer.userElapsed_;
      sysElapsed_ = timer.sysElapsed_;
      realElapsed_ = timer.realElapsed_;
      enabled_ = timer.enabled_;
   }
   return *this;
}

/// ----------------------------------------------------------------------------
CTimerN &CTimerN::start(void)
{
   if (!enabled_) {
      realStart_ = currentTime();
      gettimeofday(&rS, NULL);
      getCPUTime(&userStart_, &sysStart_);
      enabled_ = true;
   }
   return *this;
}

/// ----------------------------------------------------------------------------
CTimerN &CTimerN::stop(void)
{
   if (enabled_) {
      time_ms_t tu;
      time_ms_t ts;
      getCPUTime(&tu, &ts);
      userElapsed_ += tu - userStart_;
      sysElapsed_ += ts - sysStart_;
      gettimeofday(&rE, NULL);
      realElapsed_ +=
          ((rE.tv_sec - rS.tv_sec) * 1000 + (rE.tv_usec - rS.tv_usec) / 1000);
      enabled_ = false;
   }
   return *this;
}

/// ----------------------------------------------------------------------------
CTimerN &CTimerN::reset(void)
{
   userElapsed_ = 0;
   sysElapsed_ = 0;
   realElapsed_ = 0;
   enabled_ = false;
   return *this;
}

/// ----------------------------------------------------------------------------
void CTimerN::restart(void)
{
   reset();
   start();
}

/// ----------------------------------------------------------------------------
bool CTimerN::enabled(void) const { return enabled_; }

/// ----------------------------------------------------------------------------
long CTimerN::realCurTime(void) const { return (currentTime() - realStart_); }

/// ----------------------------------------------------------------------------
long CTimerN::realTime(void) const { return realElapsed_; }

/// ----------------------------------------------------------------------------
long CTimerN::sysTime(void) const { return sysElapsed_; }

/// ----------------------------------------------------------------------------
long CTimerN::userTime(void) const { return userElapsed_; }

/// ----------------------------------------------------------------------------
long CTimerN::cpuTime(void) const { return userElapsed_ + sysElapsed_; }

/// ----------------------------------------------------------------------------
CTimerN &CTimerN::append(const CTimerN &timer)
{
   realElapsed_ += timer.realTime();
   userElapsed_ += timer.userTime();
   sysElapsed_ += timer.sysTime();
   return *this;
}

/// ----------------------------------------------------------------------------
CTimerN &CTimerN::append(long (&t)[3])
{
   realElapsed_ += t[0];
   userElapsed_ += t[1];
   sysElapsed_ += t[2];
   return *this;
}

/// ----------------------------------------------------------------------------
void CTimerN::addTime(long (&t)[3]) { addTime(t, *this); }

/// ----------------------------------------------------------------------------
/// Static methods

/// - static method ------------------------------------------------------------
long CTimerN::getTime(void)
{
   struct timeval t;
   gettimeofday(&t, NULL);
   return t.tv_sec;
}

/// - static method ------------------------------------------------------------
long CTimerN::getTimeMS(void)
{
   struct timeval t;
   gettimeofday(&t, NULL);
   return t.tv_sec * 1000 + t.tv_usec / 1000;
}

/// - static method ------------------------------------------------------------
CTimerN::time_ms_t CTimerN::currentTime(void)
{
   struct timeval t;
   gettimeofday(&t, NULL);
   return time_ms_t(t.tv_sec) * time_ms_t(1000) + t.tv_usec / 1000;
}

/// - static method ------------------------------------------------------------
double CTimerN::getUserTime(bool self)
{
   struct rusage usage;
   double ret = 0.0;
   int r = getrusage(self ? RUSAGE_SELF : RUSAGE_CHILDREN, &usage);
   if (r == 0) {
      ret = double(usage.ru_utime.tv_sec)                 // seconds
            + double(usage.ru_utime.tv_usec) / 1000000.0; // microseconds
   }
   return ret;
}

/// - static method ------------------------------------------------------------
double CTimerN::getSysTime(bool self)
{
   struct rusage usage;
   double ret = 0.0;
   int r = getrusage(self ? RUSAGE_SELF : RUSAGE_CHILDREN, &usage);
   if (r == 0) {
      ret = double(usage.ru_stime.tv_sec)                 // seconds
            + double(usage.ru_stime.tv_usec) / 1000000.0; // microseconds
   }
   return ret;
}

/// - static method ------------------------------------------------------------
bool CTimerN::getCPUTime(double *cpu, double *user, double *sys, bool self)
{
   struct rusage usage;
   int r = getrusage(self ? RUSAGE_SELF : RUSAGE_CHILDREN, &usage);
   if (r == 0) {
      *user =
          double(usage.ru_utime.tv_sec) + usage.ru_utime.tv_usec / 1000000.0;
      *sys = double(usage.ru_stime.tv_sec) + usage.ru_stime.tv_usec / 1000000.0;
      *cpu = *user + *sys;
   }
   return r == 0;
}

/// - static method ------------------------------------------------------------
bool CTimerN::getCPUTime(time_ms_t *user, time_ms_t *sys, bool self)
{
   struct rusage usage;
   int r = getrusage(self ? RUSAGE_SELF : RUSAGE_CHILDREN, &usage);
   if (r == 0) {
      *user = usage.ru_utime.tv_sec * (time_ms_t)1000;
      *user += usage.ru_utime.tv_usec / 1000;
      *sys = usage.ru_stime.tv_sec * (time_ms_t)1000;
      *sys += usage.ru_stime.tv_usec / 1000;
   }
   return r == 0;
}

/// - static method ------------------------------------------------------------
void CTimerN::resetRealClock(void)
{
   struct timeval t;
   gettimeofday(&t, NULL);
   processStartSec = t.tv_sec;
   processStartUSec = t.tv_usec;
}

/// - static method ------------------------------------------------------------
long CTimerN::getRTimeMS(void)
{
   struct timeval t;
   gettimeofday(&t, NULL);
   t.tv_sec -= processStartSec;
   t.tv_usec -= processStartUSec;
   if (t.tv_usec < 0) {
      t.tv_sec -= 1;
      t.tv_usec += 1000000;
   }
   return t.tv_sec * 1000 + t.tv_usec / 1000;
}

/// - static method ------------------------------------------------------------
long CTimerN::toMS(long sec, long usec) { return sec * 1000 + usec / 1000; }

/// - static method ------------------------------------------------------------
void CTimerN::sleep(long ms) { usleep(ms * 1000); }

/// - static method ------------------------------------------------------------
void CTimerN::addTime(long (&t)[3], crl::CTimerN &timer)
{
   t[0] += timer.realTime();
   t[1] += timer.cpuTime();
   t[2] += timer.userTime();
}

/* end of timerN.cc */
