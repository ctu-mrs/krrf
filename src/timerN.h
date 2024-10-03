/*
 * File name: timerN.h
 * Date:      2005/10/31 08:31
 * Author:    Jan Faigl
 *
 * 2007/03/29 - Rewrite to use real, sys, user and cpu (user+sys) time
 *              time is now accumulated, start and stop, increment particular
 * times all times are in ms (long type)
 */

#ifndef __TIMERN_H__
#define __TIMERN_H__

#include <sys/time.h>

#include <inttypes.h>

namespace crl {

class CTimerN {
 public:
   typedef int64_t time_ms_t;

 private:
   time_ms_t userStart_;
   time_ms_t sysStart_;
   time_ms_t realStart_;

   struct timeval rS;
   struct timeval rE;

   time_ms_t userElapsed_;
   time_ms_t sysElapsed_;
   time_ms_t realElapsed_;
   bool enabled_;

   static long processStartSec;
   static long processStartUSec;

 public:
   CTimerN();
   CTimerN(const CTimerN &timer);

   CTimerN &operator=(const CTimerN &pt);

   CTimerN &start(void);
   CTimerN &stop(void);
   CTimerN &reset(void);

   CTimerN &suspend(void) { return stop(); }

   CTimerN &resume(void) { return start(); }

   /// ----------------------------------------------------------------------------
   /// @brief realCurTime
   ///
   /// @return current real timer time in ms
   /// @deprecated as long (32 bits) is too small to store epoch time in ms
   /// ----------------------------------------------------------------------------
   long realCurTime(void) const;

   /// ----------------------------------------------------------------------------
   /// @brief restart shortcut for reset, start
   /// ----------------------------------------------------------------------------
   void restart(void);

   bool enabled(void) const;

   /// ----------------------------------------------------------------------------
   /// @brief realTime
   ///
   /// @return elapsed real time in ms
   /// ----------------------------------------------------------------------------
   long realTime(void) const;

   /// ----------------------------------------------------------------------------
   /// @brief sysTime
   ///
   /// @return process elapsed sys time ms
   /// ----------------------------------------------------------------------------
   long sysTime(void) const;

   /// ----------------------------------------------------------------------------
   /// @brief userTime
   ///
   /// @return process elapsed user time ms
   /// ----------------------------------------------------------------------------
   long userTime(void) const;

   /// ----------------------------------------------------------------------------
   /// @brief cpuTime
   ///
   /// @return total process elapsed time (user+sys)
   /// ----------------------------------------------------------------------------
   long cpuTime(void) const;

   /// --------------------------------------------------------------------------
   /// @brief userTime
   ///
   /// @return user time in second
   /// ----------------------------------------------------------------------------
   //     double userTime(void);

   CTimerN &append(const CTimerN &timer);

   CTimerN &append(long (&t)[3]);

   void addTime(long (&t)[3]);

   /// ----------------------------------------------------------------------------
   /// @brief getTime
   ///
   /// @return in seconds
   /// ----------------------------------------------------------------------------
   static long getTime(void);

   /// @deprecated as long (32 bits) is too small to store epoch time in ms
   ///             used currentTime instead
   static long getTimeMS(void);

   /// ----------------------------------------------------------------------------
   /// @brief currentTime
   /// ----------------------------------------------------------------------------
   static time_ms_t currentTime(void);

   static double getUserTime(bool self = true);

   static double getSysTime(bool self = true);

   static bool getCPUTime(double *cpu, double *user, double *sys,
                          bool self = true);

   static bool getCPUTime(time_ms_t *user, time_ms_t *sys, bool self = true);

   /// ----------------------------------------------------------------------------
   /// @brief resetRealClock set processStartSec and processStartUSec to current
   /// time
   /// ----------------------------------------------------------------------------
   static void resetRealClock(void);

   /// ----------------------------------------------------------------------------
   /// @brief getRTimeMS
   ///
   /// @return difference from processStart time  in MS
   /// if resetRealClock is not called before, than it returns same value as
   /// getTimeMS()
   /// ----------------------------------------------------------------------------
   static long getRTimeMS(void);

   static long toMS(long sec, long usec);

   static void sleep(long ms);

   static void addTime(long (&t)[3], crl::CTimerN &timer);
};

} // namespace crl

#endif

/* end of timerN.h */
