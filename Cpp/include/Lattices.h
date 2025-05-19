//
// Created by itai on 7/15/24.
//

// #ifndef LATTICES_H
// #define LATTICES_H
#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <random>
#include <thread>
// #include <Eigen/src/Core/Matrix.h>
// #include <Eigen/src/Core/functors/TernaryFunctors.h>
#include <float.h>
#include <map>
#include <Eigen/Dense>


namespace Lattices {
    enum LatticeType {
        Zn,
        DnStar,
        AnStar,
        File
    };

    class Lattice {
    public:
        Lattice(int dim, double delta, double epsilon, LatticeType type);
        void setBounds(std::vector<double> * bounds);

                    class TVectorEquals {
            public:
                bool operator()(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2) const {
                    bool res = true;
                    for (int i = 0; i < v1.size(); ++i) {
                        res = res && std::fabs(v1[i] - v2[i]) < 0.0001;
                    }
                    return res;
                }
            };

            // Hash function for Eigen matrix and vector.
            // The code is from `hash_combine` function of the Boost library. See
            // http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
            template<typename T>
            struct matrix_hash : std::unary_function<T, size_t> { // moo!!
                std::size_t operator()(T const& matrix) const {
                    // Note that it is oblivious to the storage order of Eigen matrix (column- or
                    // row-major). It will give you the same hash value for two different matrices if they
                    // are the transpose of each other in different storage order.
                    size_t seed = 0;
                    for (size_t i = 0; i < matrix.size(); ++i) {
                        // auto elem = *(matrix.data() + i);
                        // auto roundElem = roundf(*(matrix.data() + i) * 10000) / 10000;
                        int roundElem = round(*(matrix.data() + i) * 10000);
                        // int elem = (int)(10000 * roundElem);
                        // std::cout << "elem:" << elem << ",";
                        seed ^= std::hash<typename T::Scalar>()(roundElem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                    }
                    // std::cout << ",  ~~~seed=" << seed << std::endl;
                    return seed;
                }
            };

            static double distEuclid(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, int d_) {
                double res = 0;
                for (int w = 0; w < d_; ++w) {
                    res += std::pow(v1[w] - v2[w], 2);
                }
                return std::sqrt(res);
            }

            static bool compareDoubles(double a, double b) {
                return fabs(a - b) < DBL_EPSILON;
            }

            static std::string EigenToString(const Eigen::VectorXd& v){
                std::string res = "";
                for (int i = 0; i < v.size(); ++i) {
                    int valint = std::round(v(i) * pow(10, 6));
                    double val = valint / pow(10, 6);
                    res += std::to_string(val) + ",";
                }
                return res;
            }


        std::tuple<int, double, double> goOverSamples(int type = 0, double r = 1, bool createFile = false);
        long getSamplesInBall(bool createFile = false, int type = 0);

        std::tuple<long, double> getSampleCountInBallNew();

        long getSampleCountInBall();
    protected:
        int d_;
        double delta_;
        double epsilon_;
        double r_;
        double rescale_;
        double max_ind_;
        std::vector<double>* bounds_;
        std::mutex mtx;
        Eigen::MatrixXd T_;
    };

    class Zn : public Lattice {
    public:
        Zn(int dim, double delta, double epsilon) : Lattice(dim, delta, epsilon, LatticeType::Zn) {
            rescale_ = (delta * epsilon) / std::sqrt(1 + pow(epsilon, 2));
            // max_ind_ = ceil(r_ / rescale_);
            max_ind_ = ceil(r_ / (2.0*rescale_));
            // Zn rescaled generator
            T_ = Eigen::MatrixXd(dim, dim);
            for (int j = 0; j < dim; ++j) {
                for (int w = 0; w < dim; ++w) {
                    if (j == w) {
                        T_(j, w) = rescale_;
                    } else {
                        T_(j, w) = 0;
                    }
                }
            }
            gen_ = std::mt19937{seed_()};
            dist_ = std::uniform_int_distribution<>{0, max_ind_};
        }

        std::vector<double> sample();
    private:
        std::random_device seed_;
        std::mt19937 gen_; // seed the generator
        std::uniform_int_distribution<> dist_; // set min and max
        int max_ind_;
    };
    //
    // class DnStar : public Lattice {
    // public:
    //     DnStar(int dim, double delta, double epsilon) : Lattice(dim, delta, epsilon) {
    //         double beta = (delta * epsilon) / std::sqrt(1 + pow(epsilon, 2));
    //         if (dim % 2 == 0) {
    //             rescale_ = sqrt(8.0 / dim) * beta;
    //         } else {
    //             rescale_ =  (4*beta) / sqrt(2.0 * dim - 1);
    //         }
    //         max_ind_ = ceil(sqrt(r_2_) / rescale_);
    //         // DnStar rescaled generator
    //         T_ = Eigen::MatrixXd(dim, dim);
    //         for (int j = 0; j < dim; ++j) {
    //             for (int w = 0; w < dim; ++w) {
    //                 if (j < dim - 1) {
    //                     if (j == w) {
    //                         T_(j, w) = rescale_;
    //                     } else {
    //                         T_(j, w) = 0;
    //                     }
    //                 } else {
    //                     T_(j, w) = rescale_ * 0.5;
    //                 }
    //             }
    //         }
    //     }
    //
    //     std::vector<double> sample();
    // private:
    //
    // };
    //
    // class AnStar : public Lattice {
    // public:
    //     AnStar(int dim, double delta, double epsilon) : Lattice(dim, delta, epsilon) {
    //         double beta = (delta * epsilon) / std::sqrt(1 + pow(epsilon, 2));
    //         rescale_ = sqrt((12.0 * (dim + 1)) / (dim * (dim + 2))) * beta;
    //         double cubeSizeFix = sqrt(12.0 / (dim + 2)) * beta;
    //         max_ind_ = ceil(sqrt(r_2_) / cubeSizeFix);
    //         // DnStar rescaled generator
    //         T_ = Eigen::MatrixXd(dim, dim);
    //         double anstar_x = 1.0 / (dim + 1 - sqrt(dim + 1));
    //         for (int j = 0; j < dim; ++j) {
    //             for (int w = 0; w < dim; ++w) {
    //                 if (j == 0) {
    //                     if (w < dim - 1) {
    //                         T_(j, w) = rescale_;
    //                     } else {
    //                         T_(j, w) = rescale_ * (anstar_x - 1);
    //                     }
    //                 } else {
    //                     if (w == j - 1) {
    //                         T_(j, w) = -rescale_;
    //                     } else if (w < dim - 1) {
    //                         T_(j, w) = 0;
    //                     } else {
    //                         T_(j, w) = rescale_ * anstar_x;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //
    //     std::vector<double> sample();
    // private:
    //
    // };

}



// #endif //LATTICES_H
