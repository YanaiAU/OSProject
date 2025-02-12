#ifndef MST_ALGORITHM_FACTORY_H
#define MST_ALGORITHM_FACTORY_H

#include "mst_algorithm.h"
#include "prim_algorithm.h"
#include "kruskal_algorithm.h"

enum AlgorithmType { PRIM, KRUSKAL };

class MSTAlgorithmFactory {
public:
    static MSTAlgorithm* createAlgorithm(AlgorithmType type);
};

#endif // MST_ALGORITHM_FACTORY_H
