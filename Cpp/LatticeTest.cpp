//
// Created by itai on 10/14/24.
//

#include "LatticeTest.h"

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

LatticeTest::LatticeTest() {
    int dim = 6;
    double delta = 1;
    double epsilon = 10;
    double beta = (delta * epsilon) / std::sqrt(1 + pow(epsilon, 2));
    double rescale_ = std::sqrt((12.0 * (dim + 1)) / (dim * (dim + 2))) * beta;
    double anstar_x = 1.0 / (dim + 1 - std::sqrt(dim + 1));
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
}

void LatticeTest::latticeAstar() {

}


void LatticeTest::goOverSamples(int type, Eigen::VectorXd& root) {
    int d_ = 6;
    std::vector<Eigen::VectorXd> open;
    std::vector<Eigen::VectorXd> openTemp;
    std::unordered_map<std::string, int> visited;
    // long samples = 1;
    // add root
    visited[EigenToString(root)] = 0;
    open.push_back(root);
    Vertex rootVertex = EigenVecToVertex_[EigenToString(root)];
    // get the samples in the cube
    while (!open.empty() || !openTemp.empty()) {
        if (open.empty()) {
            // we have the next wave of neighbors
            open = openTemp;
            openTemp.clear();
        }
        Eigen::VectorXd v = open[open.size() - 1];
        open.pop_back();
        std::string vstr = EigenToString(v);
        int sign = -1;
        for (int j = 0; j < 2; ++j) {
            sign = -sign;
            for (int i = 0; i < d_; ++i) {
                Eigen::VectorXd newNeighbor = v + sign * T_.col(i);
                std::string newNeighStr = EigenToString(newNeighbor);
                if (visited.contains(newNeighStr)) {
                    visited[newNeighStr]++;
                    // nodes can only have up to 2d visits
                    if (visited[newNeighStr] == 2 * d_) visited.erase(newNeighStr);
                } else {
                    visited[newNeighStr] = 1;
                    double newvNorm = 0;
                    bool inArea = true;
                    // is it in the rectangle
                    for (int w = 0; w < d_; ++w) {
                        inArea = inArea && (
                            (newNeighbor[w] > mapExtent_.low[w % dimRd_] || compareDoubles(newNeighbor[w], mapExtent_.low[w  % dimRd_])) &&
                            (newNeighbor[w] < mapExtent_.high[w % dimRd_] || compareDoubles(newNeighbor[w], mapExtent_.high[w  % dimRd_])));
                    }

                    if (type == 0) {
                        newvNorm = (newNeighbor - root).norm();
                        // make sure we're also in a ball
                        inArea = inArea &&  (newvNorm < r_ || compareDoubles(newvNorm, r_));
                    }
                    if (inArea) {
                        if (type == 0) {
                            // only connect to vertices that were valid
                            if (EigenVecToVertex_.contains(newNeighStr)) {
                                // connect edges
                                Vertex neighborV = EigenVecToVertex_[newNeighStr];
                                // Vertex neighborV = EigenVecToVertex_2[newNeighbor];
                                // check the edges already
                                if (si_->checkMotion(stateProperty_[rootVertex], stateProperty_[neighborV])) {
                                    const base::Cost weight = opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
                                    // const base::Cost weight(newvNorm);//opt_->motionCost(stateProperty_[rootVertex], stateProperty_[neighborV]);
                                    const Graph::edge_property_type properties(weight);
                                    const Edge &e = boost::add_edge(rootVertex, neighborV, properties, g_).first;
                                    // edges++;
                                    edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
                                    uniteComponents(rootVertex, neighborV);
                                }
                            }
                        } if (type == 1) {
                            addMilestone(nullptr, newNeighbor);
                            // if (newNeighborVertex != nullptr) samples++;
                        }
                        // we need to explore all space, regardless of collisions.
                        // some samples may be "isolated" in terms of direct connection to another
                        // sample, but in a PRM r-ball they may end up connecting.
                        openTemp.push_back(newNeighbor);
                    }
                }
            }
        }
    }
}
