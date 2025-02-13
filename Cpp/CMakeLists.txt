cmake_minimum_required(VERSION 3.28)
project(OmplLattices)

set(CMAKE_CXX_STANDARD 23)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -msse2 -mavx2 -mfma -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -msse2 -mavx2 -mfma")
#set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse2 -mavx2 -mfma -fsanitize=address")
set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fopenmp -msse2")

include_directories("/home/itai/ompl/ompl-1.6.0/src/ompl/")
add_executable(OmplLattices main.cpp
        src/Lattices.cpp
        include/Lattices.h
        src/DeterministicSampler.cpp
        include/DeterministicSampler.h
        src/DeterministicPRM.cpp
        include/DeterministicPRM.h
        src/LatticePRM.cpp
        include/LatticePRM.h
        src/ImplicitRandomPRM.cpp
        include/ImplicitRandomPRM.h
        src/SampleLimitTerminationCondition.cpp
        include/SampleLimitTerminationCondition.h
        src/RRT.cpp
        include/RRT.h
#        src/FullPRM.cpp
#        include/FullPRM.h
        src/LazyFullPRM.cpp
        include/LazyFullPRM.h
        src/ImplicitPRM.cpp
        include/ImplicitPRM.h
#        PRM.cpp
#        PRM.h
        #        LatticeTest.cpp
#        LatticeTest.h
        #        src/MultipleTranslatingRobots.cpp
#        include/MultipleTranslatingRobots.h
        #        src/LatticePRM.cpp
        #        src/RCSPlanner.cpp
#        include/RCSPlanner.h
#        src/LatticePRM.cpp
#        include/LatticePRM.h
)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(OmplLattices PRIVATE Eigen3::Eigen)

message("ompl libraries: ${OMPL_LIBRARIES}")
#find_package(ompl-1.6 CONFIG REQUIRED)
target_link_libraries(OmplLattices PRIVATE ompl_app)
target_link_libraries(OmplLattices PRIVATE ompl_app_base)
target_link_libraries(OmplLattices PRIVATE ompl)
#target_link_libraries(OmplLattices PRIVATE ompl_app_dbg)
#target_link_libraries(OmplLattices PRIVATE ompl_app_base_dbg)
#target_link_libraries(OmplLattices PRIVATE ompl_dbg)
