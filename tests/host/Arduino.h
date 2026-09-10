#pragma once
// Minimal host substitute for the Arduino types used by the protocol codec.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
using std::isfinite;
class String {
  std::string s;
 public:
  String(const char* v = "") : s(v) {}
  String(std::string v) : s(v) {}
  size_t length() const { return s.size(); }
  const char* c_str() const { return s.c_str(); }
  char operator[](size_t i) const { return s[i]; }
  int indexOf(char c, int start = 0) const {
    auto p = s.find(c, start); return p == std::string::npos ? -1 : int(p);
  }
  String substring(int start) const { return s.substr(start); }
  String substring(int start, int end) const { return s.substr(start, end-start); }
  bool operator==(const char* rhs) const { return s == rhs; }
  void trim() {
    auto a = s.find_first_not_of(" \r\n\t");
    if (a == std::string::npos) { s.clear(); return; }
    s = s.substr(a, s.find_last_not_of(" \r\n\t")-a+1);
  }
};
class Stream {
 public:
  template<class... T> void printf(const char*, T...) {}
  template<class T> void print(T) {}
  template<class T> void println(T) {}
};
inline char* dtostrf(double n, int, unsigned precision, char* buf) {
  std::snprintf(buf, 16, "%.*f", precision, n); return buf;
}
