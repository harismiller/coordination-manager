#ifndef SYSTEM_COMPILER_H
#define SYSTEM_COMPILER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ (hash2 << 1);
    }
};

union standbyLimits {
    int generalLimit;
    std::unordered_map<std::pair<int, int>, int, pair_hash> individualLimits;

    // Constructor and destructor
    standbyLimits() : generalLimit(0) {}
    ~standbyLimits() {}
};

class SystemCompiler {
private:
    std::string haltonFilePath;

    std::unordered_map<std::pair<int, int>, int, pair_hash> gridTallies;
    std::unordered_map<std::pair<int, int>, std::string, pair_hash> labeledPoints;
    std::unordered_map<std::pair<int, int>, std::string, pair_hash> zones;
    std::unordered_map<std::pair<int, int>, int, pair_hash> occupancyGrid;

    int totalPoints = 0;
    std::pair<int, int> gridDimensions = {0, 0};

    int lookahead = 0;
    union standbyLimits standby;
    bool isGeneralLimitActive = true;

    void parseHaltonPoints();

public:
    SystemCompiler(const std::string& filePath);
    ~SystemCompiler(); // Destructor to clean up the union

    void compileSystem();

    // Lookahead management
    void setLookahead(int lookahead);
    int getLookahead() const;

    // Standby limit management
    void setStandbyLimit(int generalLimit); // Set generalLimit
    void setStandbyLimit(const std::unordered_map<std::pair<int, int>, int, pair_hash>& individualLimits); // Set individualLimits
    int getStandbyLimit() const; // Get generalLimit
    std::unordered_map<std::pair<int, int>, int, pair_hash> getStandbyLimits() const; // Get individualLimits
    bool checkGeneralLimitActive() const;

    // Other getters
    std::string getHaltonFilePath() const;
    std::unordered_map<std::pair<int, int>, int, pair_hash> getGridTallies() const;
    std::unordered_map<std::pair<int, int>, std::string, pair_hash> getLabeledPoints() const;
    int getTotalPoints() const;
    std::pair<int, int> getGridDimensions() const;
};

#endif // SYSTEM_COMPILER_H