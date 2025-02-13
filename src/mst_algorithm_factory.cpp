#include "../include/mst_algorithm_factory.h"

MSTAlgorithm* MSTAlgorithmFactory::createAlgorithm(AlgorithmType type) {
    switch (type) {
        case PRIM:
            return new PrimAlgorithm();
        case KRUSKAL:
            return new KruskalAlgorithm();
        default:
            return nullptr;
    }
}
