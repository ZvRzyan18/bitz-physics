cmake_minimum_required(VERSION 3.6.0)
project(oxb1tz)



include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/unit_tests
)


file(GLOB SRCS
	"${PROJECT_SOURCE_DIR}/include/*"
	"${PROJECT_SOURCE_DIR}/src/*"
	"${PROJECT_SOURCE_DIR}/src/oxb1tz/*"
	"${PROJECT_SOURCE_DIR}/src/oxb1tz/core2d/*"
	"${PROJECT_SOURCE_DIR}/unit_tests/*"
)



#add libs
add_library(oxb1tz
 ${SRCS}
)

target_compile_definitions(oxb1tz PRIVATE OXB_BUILD)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(MSVC)
 target_compile_options(oxb1tz PRIVATE
  $<$<COMPILE_LANGUAGE:C>:/W4>
  $<$<COMPILE_LANGUAGE:C>:/WX->
  $<$<COMPILE_LANGUAGE:C>:/O2>
 )
else()
 target_compile_options(oxb1tz PRIVATE
  $<$<COMPILE_LANGUAGE:C>:-Wall>
  $<$<COMPILE_LANGUAGE:C>:-Wextra>
  $<$<COMPILE_LANGUAGE:C>:-Werror>
  $<$<COMPILE_LANGUAGE:C>:-O3>
 )
endif()
