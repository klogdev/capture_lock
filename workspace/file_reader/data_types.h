// dataset_types.h
#ifndef DATASET_TYPES_H
#define DATASET_TYPES_H

#include <iostream>
#include <string>
#include <unordered_map>

enum Dataset {Kitti, Colmap};


inline Dataset getDatasetFromName(const std::string& name) {
    static const std::unordered_map<std::string, Dataset> datasetMap = {
        {"kitti", Dataset::Kitti},
        {"colmap", Dataset::Colmap}
    };

    auto it = datasetMap.find(name);
    if (it != datasetMap.end()) {
        return it->second;
    } else {
        throw std::invalid_argument("Unknown dataset");
    }
}




#endif // DATASET_TYPES_H
