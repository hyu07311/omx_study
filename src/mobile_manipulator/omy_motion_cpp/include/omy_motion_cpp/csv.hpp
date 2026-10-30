// 벤치마크 입출력 유틸 (CSV 읽기, 통계, 시간 측정)
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace omy_motion
{

// 헤더가 있는 CSV → 열 이름으로 접근하는 행 목록
struct CsvTable
{
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;

  int col(const std::string & name) const
  {
    const auto it = std::find(header.begin(), header.end(), name);
    if (it == header.end()) {
      throw std::runtime_error("no column " + name);
    }
    return static_cast<int>(it - header.begin());
  }
  double num(size_t row, const std::string & name) const { return std::stod(rows[row][col(name)]); }
  const std::string & str(size_t row, const std::string & name) const { return rows[row][col(name)]; }
};

inline std::vector<std::string> splitCsvLine(std::string line)
{
  // Python csv 모듈은 줄 끝이 \r\n
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

inline CsvTable readCsv(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open " + path);
  }
  CsvTable t;
  std::string line;
  std::getline(f, line);
  t.header = splitCsvLine(line);
  while (std::getline(f, line)) {
    if (!line.empty() && line != "\r") {
      t.rows.push_back(splitCsvLine(line));
    }
  }
  return t;
}

inline std::string readFile(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// numpy.percentile 과 같은 선형 보간
inline double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {
    return NAN;
  }
  std::sort(v.begin(), v.end());
  const double idx = p / 100.0 * (v.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(idx));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (v[hi] - v[lo]) * (idx - lo);
}

inline double median(const std::vector<double> & v) { return percentile(v, 50.0); }

class Stopwatch
{
public:
  Stopwatch() : t0_(std::chrono::steady_clock::now()) {}
  double ms() const
  {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_).count();
  }

private:
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace omy_motion
