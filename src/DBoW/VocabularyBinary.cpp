#include <DBoW/VocabularyBinary.hpp>
#include <opencv2/core/core.hpp>
using namespace std;

namespace vins::loop_fusion {

Vocabulary::Vocabulary()
    : nNodes(0), nWords(0), nodes(nullptr), words(nullptr) {}

Vocabulary::~Vocabulary() {
  if (nodes != nullptr) {
    delete[] nodes;
    nodes = nullptr;
  }

  if (words != nullptr) {
    delete[] words;
    words = nullptr;
  }
}

void Vocabulary::serialize(ofstream &stream) {
  stream.write((const char *)this, staticDataSize());
  stream.write((const char *)nodes, sizeof(Node) * nNodes);
  stream.write((const char *)words, sizeof(Word) * nWords);
}

void Vocabulary::deserialize(ifstream &stream) {
  stream.read((char *)this, staticDataSize());

  nodes = new Node[nNodes];
  stream.read((char *)nodes, sizeof(Node) * nNodes);

  words = new Word[nWords];
  stream.read((char *)words, sizeof(Word) * nWords);
}

}  // namespace vins::loop_fusion