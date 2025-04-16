#ifndef SYSTEM_COMPILER_H
#define SYSTEM_COMPILER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <utility> // for std::pair

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ (hash2 << 1); // Combine the two hashes
    }
};

class SystemCompiler {
private:
    std::string haltonFilePath;
    std::unordered_map<std::pair<int, int>, int, pair_hash> gridTallies;
    std::unordered_map<std::pair<int, int>, std::string, pair_hash> labeledPoints;
    int totalPoints = 0; // Total number of points
    std::pair<int, int> gridDimensions = {0, 0}; // Grid dimensions (max x, max y)

    void parseHaltonPoints();

public:
    SystemCompiler(const std::string& filePath);
    void compileSystem();
};

#endif // SYSTEM_COMPILER_H