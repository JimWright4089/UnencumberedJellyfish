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
StopWatch::StopWatch(int waitTime) :
  mLastTime(0),
  mWaitTime(waitTime)
{
}

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
long StopWatch::Now(void)
{
  return millis();
}

//----------------------------------------------------------------------------
//  See header file
//----------------------------------------------------------------------------
void StopWatch::SetTime(int waitTime)
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
long StopWatch::GetTimeLeft()
{
  return mWaitTime - (Now()-mLastTime);
}