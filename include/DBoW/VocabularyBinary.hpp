#ifndef VocabularyBinary_hpp
#define VocabularyBinary_hpp

#include <cstdint>
#include <fstream>
#include <string>

namespace vins::loop_fusion {
    
struct Node {
    int32_t nodeId;
    int32_t parentId;
    double weight;
    uint64_t descriptor[4];
};

struct Word {
    int32_t nodeId;
    int32_t wordId;
};

struct Vocabulary {
    int32_t k;
    int32_t L;
    int32_t scoringType;
    int32_t weightingType;
    
    int32_t nNodes;
    int32_t nWords;
    
    Node* nodes;
    Word* words;
    
    Vocabulary();
    ~Vocabulary();
    
    void serialize(std::ofstream& stream);
    void deserialize(std::ifstream& stream);
    
    inline static size_t staticDataSize() {
        return sizeof(Vocabulary) - sizeof(Node*) - sizeof(Word*);
    }
};

} // namespace vins::loop_fusion

#endif /* VocabularyBinary_hpp */
