CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(detourcrowd_SRCS
	Source/DetourPathCorridor.cpp
	Source/DetourLocalBoundary.cpp
	Source/DetourPathQueue.cpp
	Source/DetourCrowd.cpp
	Source/DetourCollisionAvoidance.cpp
	Source/DetourPathFollowing.cpp
	Source/DetourFlockingBehavior.cpp
	Source/DetourSeparationBehavior.cpp
	Source/DetourCohesionBehavior.cpp
	Source/DetourAlignmentBehavior.cpp
	Source/DetourGoToBehavior.cpp
	Source/DetourSeekBehavior.cpp
	Source/DetourPipelineBehavior.cpp
	Source/DetourBehavior.cpp
	Source/DetourSkirtBehavior.cpp
)

SET(detourcrowd_HDRS
	Include/DetourPathCorridor.h
	Include/DetourCrowd.h
	Include/DetourLocalBoundary.h
	Include/DetourPathQueue.h
	Include/DetourCollisionAvoidance.h
	Include/DetourPathFollowing.h
	Include/DetourSeekBehavior.h
	Include/DetourFlockingBehavior.h
	Include/DetourSeparationBehavior.h
	Include/DetourCohesionBehavior.h
	Include/DetourAlignmentBehavior.h
	Include/DetourGoToBehavior.h
	Include/DetourSteeringBehavior.h
	Include/DetourBehavior.h
	Include/DetourPipelineBehavior.h
	Include/DetourParametrizedBehavior.h
	Include/DetourSkirtBehavior.h
)

INCLUDE_DIRECTORIES(Include 
	../Detour/Include
	../DetourTileCache
	../Recast/Include
)

ADD_LIBRARY(DetourCrowd ${detourcrowd_SRCS} ${detourcrowd_HDRS})
IF(IOS)
	# workaround a bug forbidding to install the built library (cf. http://www.cmake.org/Bug/view.php?id=12506)
	SET_TARGET_PROPERTIES(DetourCrowd PROPERTIES 
		ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/Debug"
		ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/Release")
ENDIF()

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(BUILD_RECASTDETOUR_FAT_LIBRARIES "Building binaries containing both 32 and 64 bits versions." 
	ON "APPLE;NOT IOS" 
	OFF)
IF (BUILD_RECASTDETOUR_FAT_LIBRARIES)
	SET_TARGET_PROPERTIES(DetourCrowd PROPERTIES OSX_ARCHITECTURES "i386;x86_64")
ENDIF()

# To fix compilation problem: relocation R_X86_64_32 against `a local symbol' can not be
# used when making a shared object; recompile with -fPIC
# See http://www.cmake.org/pipermail/cmake/2007-May/014350.html
IF( UNIX )
	SET_TARGET_PROPERTIES(DetourCrowd PROPERTIES COMPILE_FLAGS "-fPIC")
ENDIF()

INSTALL(FILES ${detourcrowd_HDRS}
	DESTINATION include/detourCrowd
)

INSTALL(TARGETS DetourCrowd EXPORT "RecastDetourTargets-${PLATFORM_STR}"
	RUNTIME DESTINATION bin/${PLATFORM_STR}
	LIBRARY DESTINATION lib/${PLATFORM_STR}
	ARCHIVE DESTINATION lib/${PLATFORM_STR}
)

if(WIN32)
	set(CMAKE_CXX_FLAGS "/EHsc /GR- /W4")
else(WIN32)
	set(CMAKE_CXX_FLAGS "-Wall -ansi -O3")
endif(WIN32)
