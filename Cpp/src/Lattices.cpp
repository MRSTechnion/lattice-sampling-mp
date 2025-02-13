//
// Created by itai on 7/15/24.
//

#include "../include/Lattices.h"

#include <complex>
#include <iostream>

#include <list>
#include <random>
#include <string>
#include <vector>

#include <chrono>
#include <future>
#include <float.h>
#include <fstream>

using namespace std::chrono_literals;

void getIntegerVectors(int max_val, int dim, std::vector<std::vector<double> *> &powerSetRes, int i) {
    std::vector<std::vector<double>*> tmp;
    bool start = false;
    if (powerSetRes.empty()) {
        powerSetRes.push_back(new std::vector<double>);
        start = true;
    }
    for (std::vector<double>* set: powerSetRes) {
        for (int j = 0; j < max_val; j++) {
            auto* new_set = new std::vector<double>(*set);
            new_set->insert(new_set->begin(), j);
            tmp.push_back(new_set);
        }
    }
    if (start) powerSetRes.clear();
    for (std::vector<double>* set: tmp) {
        if (set->size() == i) powerSetRes.push_back(set);
    }
    auto it = powerSetRes.begin();
    while(it != powerSetRes.end()) {
        if((*it)->size() < i) {
            auto toDelete = *it;
            it = powerSetRes.erase(it);
            delete toDelete;
        }
        else {
            ++it;
        }
    }
    if (i < dim) getIntegerVectors(max_val, dim, powerSetRes, i + 1);
}

static std::string toString(const Eigen::VectorXd& v){
    std::string res = "";
    for (int i = 0; i < v.size(); ++i) {
        int valint = std::round(v(i) * pow(10, 6));
        double val = valint / pow(10, 6);
        res += std::to_string(val) + ",";
    }
    return res;
}

bool areDoubleEq(double a, double b) {
    return fabs(a - b) < DBL_EPSILON;
}

std::vector<std::string>* getSampleLines(const std::string &bin) {
    // Open the binary file for reading
    std::ifstream inputFile(bin, std::ios::binary);

    if (!inputFile) {
        std::cerr << "Error opening file for reading!" << std::endl;
        return nullptr;
    }

    // Vector to store the lines as strings
    auto* lines = new std::vector<std::string>;

    // Buffer to store each "line"
    std::string currentLine;
    char c;
    // Read the file character by character
    while (inputFile.get(c)) {
        // If we encounter a newline character, we've reached the end of a "line"
        if (c == '\n') {
            lines->push_back(currentLine); // Save the current line to the vector
            currentLine.clear();          // Clear the buffer for the next line
        } else {
            currentLine += c; // Add character to the current line
        }
    }

    // Handle any remaining data if the file does not end with a newline
    if (!currentLine.empty()) {
        lines->push_back(currentLine); // Save the last line
    }

    // Close the file
    inputFile.close();

    return lines;
}

struct LatticeNN {
    Eigen::VectorXd sample;
    Eigen::VectorXi sampleI;
    double g;
    bool isGoal = false;
};

template<typename T>
struct matrix_hashI : std::unary_function<T, size_t> { // moo!!
    std::size_t operator()(T const& matrix) const {
        // Note that it is oblivious to the storage order of Eigen matrix (column- or
        // row-major). It will give you the same hash value for two different matrices if they
        // are the transpose of each other in different storage order.
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            // auto roundElem = roundf(*(matrix.data() + i) * 10000) / 10000;
            // int roundElem = round(*(matrix.data() + i) * 10000);
            // int elem = (int)(10000 * roundElem);
            // std::cout << "elem:" << elem << ",";
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        // std::cout << ",  ~~~seed=" << seed << std::endl;
        return seed;
    }
};

class TVectorEqualsI {
public:
    bool operator()(const Eigen::VectorXi& v1, const Eigen::VectorXi& v2) const {
        for (int i = 0; i < v1.size(); ++i) {
            if (v1[i] != v2[i]) return false;
        }
        return true;
    }
};

std::tuple<long, double> Lattices::Lattice::getSampleCountInBallNew() {
    int d_ = this->d_;
    std::vector<LatticeNN> open;
    std::vector<LatticeNN> openTemp;
    std::unordered_map<Eigen::VectorXi, int, matrix_hashI<Eigen::VectorXi>, TVectorEqualsI> visited;
    Eigen::VectorXd root = Eigen::VectorXd::Zero(d_);
    Eigen::VectorXi currI = Eigen::VectorXi::Zero(d_);

    long samples = 1;
    double sumOfEdges = 0;

    visited[currI] = 0;
    open.push_back({root, currI});

    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        const auto [sample, sampleI, g, isGoal] = open[open.size() - 1];
        open.pop_back();
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = sample + sign * T_.col(i);
                Eigen::VectorXi newNeighborI = sampleI;
                newNeighborI[i] += sign;
                if (visited.contains(newNeighborI) && visited[newNeighborI] < 2*d_) {
                    visited[newNeighborI]++;
                    if (visited[newNeighborI] == 2 * d_)
                        visited.erase(newNeighborI);
                } else {
                    // new sample
                    visited[newNeighborI] = 1;
                    double newvNorm = distEuclid(newNeighbor, root, d_);
                    if (newvNorm < r_ || compareDoubles(newvNorm, r_)) {
                        // the regular ball-search is unscaled
                        samples++;
                        sumOfEdges += newvNorm;
                        openTemp.push_back(
                            {newNeighbor, newNeighborI});
                    }
                    // if (newvNorm < 1.5 * r_ || compareDoubles(newvNorm, 1.5 * r_)) {
                    //     openTemp.push_back(
                    //         {newNeighbor, newNeighborI});
                    //     // trying to cover more points I may have missed on the edges
                    // }
                }
            }
        }
    }
    return {samples, sumOfEdges};
}

long Lattices::Lattice::getSampleCountInBall() {
    int d_ = this->d_;
    std::vector<Eigen::VectorXd> open;
    std::vector<Eigen::VectorXd> openTemp;
    std::unordered_map<Eigen::VectorXd, int, matrix_hash<Eigen::VectorXd>, TVectorEquals> visited;
    long samples = 1;
    Eigen::VectorXd root = Eigen::VectorXd::Zero(d_);
    // add root
    visited[root] = 0;
    open.push_back(root);
    std::vector<Eigen::VectorXd> result;
    Eigen::VectorXd bla;
    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        Eigen::VectorXd v = open[open.size() - 1];
        open.pop_back();
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
                std::string newNeighStr = EigenToString(newNeighbor);

                if (visited.contains(newNeighbor) && visited[newNeighbor] < 2*d_) {
                    visited[newNeighbor]++;
                    if (visited[newNeighbor] == 2 * d_)
                        visited.erase(newNeighbor);
                } else {
                    visited[newNeighbor] = 1;
                    double newvNorm = 0;
                    bool inArea = true;
                    newvNorm = distEuclid(newNeighbor, root, d_);
                    inArea = inArea &&  (newvNorm < r_ || compareDoubles(newvNorm, r_));
                    if (inArea) {
                        std::cout << EigenToString(newNeighbor) << std::endl;
                        samples++;
                        result.push_back(newNeighbor);
                        openTemp.push_back(newNeighbor);
                    }
                }
            }
        }
    }
    return samples;
}

Lattices::Lattice::Lattice(int dim, double delta, double epsilon, LatticeType type):
    d_(dim), delta_(delta), epsilon_(epsilon), bounds_(nullptr), r_(1) {
    double beta = (delta * epsilon) / std::sqrt(1 + pow(epsilon, 2));
    // r = (r == -1 ? 1 :
    r_ = (2 * delta * (epsilon + 1)) / std::sqrt(1 + pow(epsilon, 2));
    // r_ = 5;
    std::cout << "Lattice radius=" << r_ << std::endl;
    switch (type) {
        case LatticeType::File:
            break;
        case LatticeType::Zn:
            rescale_ = (2 * beta) / sqrt(dim);
            max_ind_ = ceil(2*r_ / rescale_);
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
            break;
        case LatticeType::DnStar:
            if (dim % 2 == 0) {
                rescale_ = sqrt(8.0 / dim) * beta;
            } else {
                rescale_ =  (4*beta) / sqrt(2.0 * dim - 1);
            }
            max_ind_ = 3*ceil(2*r_ / rescale_);
            // DnStar rescaled generator
            T_ = Eigen::MatrixXd(dim, dim);
            for (int j = 0; j < dim; ++j) {
                for (int w = 0; w < dim; ++w) {
                    if (j < dim - 1) {
                        if (j == w) {
                            T_(j, w) = rescale_;
                        } else {
                            T_(j, w) = 0;
                        }
                    } else {
                        T_(j, w) = rescale_ * 0.5;
                    }
                }
            }
            T_ = T_.transpose().eval();
        // std::cout << T_ << std::endl;
            break;
        case LatticeType::AnStar:
            rescale_ = sqrt((12.0 * (dim + 1)) / (dim * (dim + 2))) * beta;
            // double cubeSizeFix = sqrt(12.0 / (dim + 2)) * beta;
            // max_ind_ = 6*ceil(sqrt(r_ * r_) / cubeSizeFix);
            double anstar_x = 1.0 / (dim + 1 - sqrt(dim + 1));
            double fix = 1.0 / std::min(anstar_x, 1-anstar_x);
            max_ind_ = ceil(2*fix * (r_ / rescale_));
            // DnStar rescaled generator
            T_ = Eigen::MatrixXd(dim, dim);
            for (int j = 0; j < dim; ++j) {
                for (int w = 0; w < dim; ++w) {
                    if (j == 0) {
                        if (w < dim - 1) {
                            T_(j, w) = rescale_;
                        } else {
                            T_(j, w) = rescale_ * (anstar_x - 1);
                        }
                    } else {
                        if (w == j - 1) {
                            T_(j, w) = -rescale_;
                        } else if (w < dim - 1) {
                            T_(j, w) = 0;
                        } else {
                            T_(j, w) = rescale_ * anstar_x;
                        }
                    }
                }
            }
            break;
    }
}

void Lattices::Lattice::setBounds(std::vector<double> *bounds) {
    bounds_ = bounds;
}


std::vector<double> Lattices::Zn::sample() {
    std::vector<double> res;// = new std::vector<double>();
    for (int i = 0; i < d_; i++) {
        int val = dist_(gen_);
        res.push_back(rescale_ * (val - (max_ind_ / 2)));
    }
    return res;
}
