
# Effective Sampling for Robot Motion Planning Through the Lens of Lattices

This repository contains the code used to run the experiments for the paper. It was accepted to RSS 2025, and this version contains the sum of work that this paper included.

* [RSS 2025](https://roboticsconference.org/program/papers/48/).
* [ArXiv](https://arxiv.org/abs/2502.04908)

| ![UM_scenarios.png](https://github.com/user-attachments/assets/e3f8765f-6441-442c-9174-8381a05413dc) | ![sleeve_manipulator.png](https://github.com/user-attachments/assets/3ed5eccc-fe39-4ced-ac05-c54a0aeeb45d) |
|:--:|:--:|
| *Example scenario with multiple translated disks in a maze. Implemented in OMPL.* | * Example scenario relying on the VAMP library.*  |

## Authors

- [@Itai Panasoff, Technion](https://ItaiRobotics.github.io)
- [@Kiril Solovey, Technion](https://kirilsol.github.io/)

We are both from the [@MRS lab](https://mrstechnion.github.io/) at the Technion.



## Paper abstract

Sampling-based methods for motion planning, which
capture the structure of the robot’s free space via (typically
random) sampling, have gained popularity due to their scalability,
simplicity, and for offering global guarantees, such as probabilis-
tic completeness and asymptotic optimality. Unfortunately, the
practicality of those guarantees remains limited as they do not
provide insights into the behavior of motion planners for a finite
number of samples (i.e., a finite running time). In this work,
we harness lattice theory and the concept of (δ, ε)-completeness
by Tsao et al. (2020) to construct deterministic sample sets
that endow their planners with strong finite-time guarantees
while minimizing running time. In particular, we introduce a
highly-efficient deterministic sampling approach based on the $A_d^*$
lattice, which is the best-known geometric covering in dimensions
≤ 21. Using our new sampling approach, we obtain at least
an order-of-magnitude speedup over existing deterministic and
uniform random sampling methods for complex motion-planning
problems. Overall, our work provides deep mathematical insights
while advancing the practical applicability of sampling-based
motion planning.

## Code
### Prerequisits

To successfully run the code (as tested on Linux), several things are needed.

* OMPL: The [open motion-planning library](https://ompl.kavrakilab.org/) is the motion planning framework with which we write the tests. Follow the instructions on the site to install it. Afterwards, go to the CMakeLists file in the code project and change the line:
    
    `include_directories("/home/itai/ompl/ompl-1.6.0/src/ompl/")`

    so that it fits where you installed it.

* VAMP: The [VAMP](https://github.com/KavrakiLab/vamp) repo implements extremely quick manipulator planning. We showcase it in the paper, and so it is needed in the project. Follow the instructions to install it. Afterwards, go to the CMakeLists file in the code project and change the line:
    
    `target_include_directories(OmplLattices PRIVATE /home/itai/VAMP/vamp/build/_deps/pdqsort-src)`

    so that it fits where you installed it. This solves some error in compiling the code (you are welcome to just solve it!).

* Eigen3: [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) is a C++ library for performing linear-algebra operations. Installing shouldn't be an issue here.

* NTL: The [Number-Theory Library](https://libntl.org/) is used in the project for some operations (like Cholesky decomposition) we were considering for the paper, but didn't use in the end. Still, it is included an needed, as it is a baseline for future work on the matter.

### Features

* Explicit construction of the lattices in the paper (see the `CreateLatticeParameters()` function in `ImplicitPRM.cpp`). 
* The iA* algorithm as used on lattices (see the `constructSolutionImplicitlyLattice(..)` function in `ImplicitPRM.cpp`).
* Testing both on a manipulator model (see the `VAMPTests.h/cpp` files) and on multiple-moving-disks scenarios (see the `implicit_PRM_2D_multi(..)` function).

### Usage/Examples

The code contains two parts. 
* The python files mostly helped me with plotting the figures for the paper: the graphs (A as a function of B), and the lattice illustrations.
* The C++ files contain the bulk of the work in the paper. This means:
    * Comparing lattices to each other.
    * Comparing $A_d^*$ to random sets.
    * Calibrating a lattice, i.e. finding the best density (in terms of runtime).
    * VAMP tests with the $A_d^*$ lattice.

To use run the tests you can focus on the `main.cpp` file, and within it the normal `int main(int argc, char **)` function. There, you will find several test options:

```cpp
    /// Comparative test sets
    compareLatticesToEachother(folderBase);
    calibrateLattices(folderBase);
    AnstarVSImplicitPRM(folderBase);

    /// Vamp tests
    VAMPTests::runVampTests(argc, folderBase);
    /// counting samples
    findNumberOfSamples();

```

The main point of concern needs to be in the `std::vector<TestData> getTests()` function, which returns the functions you currently test. In there, you will have several `TestData` structures (representing tests), which are later added to the test list. That is where you need to choose what to add:

```cpp
    /// Zn vs Dn* vs An*
    std::vector<TestData> lattice_tests;

    /// d=6
    // lattice_tests.push_back(bugtrapT1B);
    // lattice_tests.push_back(bugtrapT2);
    // lattice_tests.push_back(bugtrapT2B);
    // lattice_tests.push_back(bugtrapT3);
    // lattice_tests.push_back(bugtrapT5);
    // lattice_tests.push_back(bugtrapT6);
    // lattice_tests.push_back(bugtrapT8);
    // lattice_tests.push_back(bugtrapT8B);
    // lattice_tests.push_back(bugtrapT11);

    lattice_tests.push_back(kennyT1);

    // lattice_tests.push_back(UniqueMaze1);
    // lattice_tests.push_back(UniqueMaze2);
    // lattice_tests.push_back(UniqueMaze4B3);
    // lattice_tests.push_back(UniqueMaze5);


    // lattice_tests.push_back(UniqueMaze3);
```

