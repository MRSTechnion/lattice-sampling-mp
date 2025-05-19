cmake_minimum_required(VERSION 3.28)
project(OmplLattices)

set(CMAKE_CXX_STANDARD 23)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -msse2 -mavx2 -mfma -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -msse2 -mavx2 -mfma -lflint -lntl -lgmp -lm")
#set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse2 -mavx2 -mfma -fsanitize=address")

#set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fopenmp -msse2 -lflint -lntl -lgmp -lm")
set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fopenmp -msse2 -mavx2 -mfma -lflint -lntl -lgmp -lm")

include_directories("/home/itai/ompl/ompl-1.6.0/src/ompl/")
add_executable(OmplLattices main.cpp
        src/Lattices.cpp
        include/Lattices.h
#        src/DeterministicSampler.cpp
#        include/DeterministicSampler.h
#        src/DeterministicPRM.cpp
#        include/DeterministicPRM.h
#        src/LatticePRM.cpp
#        include/LatticePRM.h
#        src/ImplicitRandomPRM.cpp
#        include/ImplicitRandomPRM.h
        src/SampleLimitTerminationCondition.cpp
        include/SampleLimitTerminationCondition.h
#        src/FullPRM.cpp
#        include/FullPRM.h
        src/LazyFullPRM.cpp
        include/LazyFullPRM.h
        src/ImplicitPRM.cpp
        include/ImplicitPRM.h
        src/ImplicitPRM_VAMP.cpp
        include/ImplicitPRM_VAMP.h
        src/VAMPTests.cpp
        include/VAMPTests.h
)

target_include_directories(OmplLattices PRIVATE /home/itai/VAMP/vamp/build/_deps/pdqsort-src)

find_package(Eigen3 CONFIG REQUIRED)
#find_package(fmt CONFIG REQUIRED)
target_link_libraries(OmplLattices PRIVATE Eigen3::Eigen)

message("ompl libraries: ${OMPL_LIBRARIES}")
#find_package(ompl-1.6 CONFIG REQUIRED)
target_link_libraries(OmplLattices PRIVATE ompl_app)
target_link_libraries(OmplLattices PRIVATE ompl_app_base)
target_link_libraries(OmplLattices PRIVATE ompl)
target_link_libraries(OmplLattices PRIVATE flint ntl gmp)
#target_link_libraries(OmplLattices PRIVATE fmt::fmt)
#target_link_libraries(OmplLattices PRIVATE ompl_app_dbg)
#target_link_libraries(OmplLattices PRIVATE ompl_app_base_dbg)
#target_link_libraries(OmplLattices PRIVATE ompl_dbg)
