//----------------------------------------------------------------------------
//
//  $Workfile: StopWatch.hpp
//
//  $Revision: X$
//
//  Project:    Unencumbered Jellyfish
//
//                            Copyright (c) 2022
//                               Jim Wright
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
#include "StopWatch.hpp"

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
StopWatch::StopWatch() :
  mLastTime(0),
  mWaitTime(1000)
{
}
  
//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
StopWatch::StopWatch(uint32_t waitTime) :
  mLastTime(0),
  mWaitTime(waitTime)
{
}

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
uint32_t StopWatch::Now(void)
{
  struct timeval theTime;
  gettimeofday(&theTime,NULL);
  return ((theTime.tv_sec * 1000) + (theTime.tv_usec/1000));
}

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
void StopWatch::SetTime(uint32_t waitTime)
{
  mWaitTime = waitTime;
}

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
bool StopWatch::IsExpired(void)
{
  if((Now() - mLastTime)>mWaitTime)
  {
     return true;
  }
  return false;
}

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
void StopWatch::Reset(void)
{
  mLastTime = Now();
}

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
uint32_t StopWatch::GetTimeLeft()
{
  return mWaitTime - (Now()-mLastTime);
}
