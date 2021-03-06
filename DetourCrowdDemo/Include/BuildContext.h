//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef BUILDCONTEXT_H
#define BUILDCONTEXT_H

#include "Recast.h"
#include "PerfTimer.h"

#include <cstdio>

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext : public rcContext
{
public:
	BuildContext();
	virtual ~BuildContext();
	
protected:	
	/// Virtual functions for custom implementations.
	///@{
	virtual void doLog(const rcLogCategory category, const char* msg, const int len);
	virtual void doResetTimers();
	virtual void doStartTimer(const rcTimerLabel /*label*/);
	virtual void doStopTimer(const rcTimerLabel /*label*/);
	virtual int doGetAccumulatedTime(const rcTimerLabel /*label*/) const;
	///@}

	TimeVal m_startTime[RC_MAX_TIMERS];
	int m_accTime[RC_MAX_TIMERS];
};

#endif // BUILDCONTEXT_H

