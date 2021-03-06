#
# Copyright (c) 2013 MASA Group recastdetour@masagroup.net
#
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
#    claim that you wrote the original software. If you use this software
#    in a product, an acknowledgment in the product documentation would be
#    appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
#    misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

FIND_PACKAGE(OpenGL)
FIND_PACKAGE(SDL)
INCLUDE(CMakeDependentOption)

# Options
CMAKE_DEPENDENT_OPTION(BUILD_DETOURCROWD_DEMO "Build DetourCrowd graphical demo applications" 
	ON "OPENGL_FOUND;OPENGL_GLU_FOUND;SDL_FOUND"
	OFF)

IF(BUILD_DETOURCROWD_DEMO)
	SET(detourcrowddemo_SRCS
		Source/Application.cpp
		Source/BuildContext.cpp
		Source/CrowdSample.cpp
		Source/DebugInfo.cpp
		Source/Filelist.cpp
		Source/imgui.cpp
		Source/imguiRenderGL.cpp
		Source/main.cpp
		Source/PerfTimer.cpp
		Source/SampleInterfaces.cpp
		Source/ValueHistory.cpp
		Source/Visualization.cpp
	)

	SET(detourcrowddemo_HDRS
		Include/Application.h
		Include/BuildContext.h
		Include/CrowdSample.h
		Include/DebugInfo.h
		Include/Filelist.h
		Include/imgui.h
		Include/imguiRenderGL.h
		Include/PerfTimer.h
		Include/SampleInterfaces.h
		Include/SDLMain.h
		Include/StaticConfiguration.h
		Include/ValueHistory.h
		Include/Visualization.h
	)

	SET(SimpleJSON_DIR Contrib/SimpleJSON)
	SET(SimpleJSON_SRCS 
		${SimpleJSON_DIR}/JSON.cpp
		${SimpleJSON_DIR}/JSON.h
		${SimpleJSON_DIR}/JSONValue.cpp
		${SimpleJSON_DIR}/JSONValue.h
	)

	SET(fastlz_DIR Contrib/fastlz)
	SET(fastlz_SRCS 
		${fastlz_DIR}/fastlz.c
	)

	SET(sdl_DIR Contrib/SDL)

	SOURCE_GROUP(sources FILES ${detourcrowddemo_HDRS} ${detourcrowddemo_SRCS})
	SOURCE_GROUP(externals\\fastlz FILES ${fastlz_SRCS})
	SOURCE_GROUP(externals\\SimpleJSON FILES ${SimpleJSON_SRCS})
	SOURCE_GROUP(cmake FILES CMakeLists.txt)

	IF(MSVC)
		# Look in the Contrib directory for SDL when building with MSVC
		SET(CMAKE_PREFIX_PATH ${sdl_DIR})
	ENDIF()

	INCLUDE_DIRECTORIES(Include 
		${fastlz_DIR}
		${SimpleJSON_DIR}
		${SDL_INCLUDE_DIR}
		../Detour/Include
		../DetourCrowd/Include
		../DetourNavmeshCreator/Include
		../DetourTileCache/Include
		../Recast/Include
		../RecastDetourDebugUtils/Include
		Contrib
	)

	IF(APPLE)
		ADD_EXECUTABLE(DetourCrowdDemo Source/SDLMain.m Include/SDLMain.h ${detourcrowddemo_SRCS} ${detourcrowddemo_HDRS} ${fastlz_SRCS} ${SimpleJSON_SRCS})
	ELSEIF(MSVC)
		ADD_EXECUTABLE(DetourCrowdDemo WIN32 ${detourcrowddemo_SRCS} ${detourcrowddemo_HDRS} ${fastlz_SRCS} ${SimpleJSON_SRCS})
	ELSE()
		ADD_EXECUTABLE(DetourCrowdDemo ${detourcrowddemo_SRCS} ${detourcrowddemo_HDRS} ${SimpleJSON_SRCS})
	ENDIF()

	SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY DEBUG_POSTFIX -gd)
	SET(DETOURCROWDDEMO_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Bin)
	SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${DETOURCROWDDEMO_BIN_DIR})
	SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${DETOURCROWDDEMO_BIN_DIR})
	IF(MSVC)
		# Enable some linker optimisations
		SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY LINK_FLAGS_RELEASE "/OPT:REF /OPT:ICF")
		SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY LINK_FLAGS_MINSIZEREL "/OPT:REF /OPT:ICF")
		SET_PROPERTY(TARGET DetourCrowdDemo PROPERTY LINK_FLAGS_RELWITHDEBINFO "/OPT:REF /OPT:ICF")
	ENDIF()

	TARGET_LINK_LIBRARIES(DetourCrowdDemo RecastDetourDebugUtils Detour DetourCrowd DetourNavmeshCreator DetourTileCache Recast ${SDL_LIBRARY} ${SDLMAIN_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

	INSTALL(DIRECTORY Bin/Meshes Bin/Samples
		DESTINATION run/${PLATFORM_STR}
	)

	IF(MSVC)
		# Retrieve the SDL dll directory
		GET_FILENAME_COMPONENT(SDL_LIB_DIR ${SDLMAIN_LIBRARY} PATH)
		INSTALL(
			FILES Bin/DroidSans.ttf ${SDL_LIB_DIR}/SDL.dll
			DESTINATION run/${PLATFORM_STR}
		)
	ELSE()
		INSTALL(
			FILES Bin/DroidSans.ttf
			DESTINATION run/${PLATFORM_STR}
		)
	ENDIF()

	INSTALL(TARGETS DetourCrowdDemo 
		RUNTIME DESTINATION run/${PLATFORM_STR}
		BUNDLE DESTINATION run/${PLATFORM_STR}
		LIBRARY DESTINATION lib/${PLATFORM_STR}
		ARCHIVE DESTINATION lib/${PLATFORM_STR}
	)
ENDIF()
