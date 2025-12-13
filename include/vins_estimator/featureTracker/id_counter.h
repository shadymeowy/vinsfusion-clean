#pragma once

// singleton ID counter for feature points

class IdCounter {
 public:
  static IdCounter &getInstance() {
    static IdCounter instance;
    return instance;
  }

  static int get() {
    IdCounter &counter = getInstance();
    return counter.id_++;
  }

  IdCounter() = default;
  ~IdCounter() = default;
  IdCounter(const IdCounter &) = delete;
  IdCounter &operator=(const IdCounter &) = delete;

 private:
  int id_{};
};