//
// Created by itai on 10/14/24.
//

#ifndef LATTICETEST_H
#define LATTICETEST_H

#include <Eigen/src/Core/Matrix.h>

class LatticeTest {
    LatticeTest();

private:
    void goOverSamples(int type, Eigen::VectorXd& root);
    void latticeAstar();
    Eigen::MatrixXd T_; // lattice transformation
};



#endif //LATTICETEST_H
