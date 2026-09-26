#include "sparkmax_protocol/parameter.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace sparkmax
{

ParameterGroup::ParameterGroup(
  std::map<std::string, ParameterDefinition> canonical,
  std::map<std::string, std::string> aliases)
: canonical_(std::move(canonical)), aliases_(std::move(aliases))
{
}

std::string ParameterGroup::normalize(const std::string & key)
{
  // key.strip().lower().replace("-", "_").replace(" ", "_")
  const auto first = key.find_first_not_of(" \t\r\n");
  const auto last = key.find_last_not_of(" \t\r\n");
  std::string out = first == std::string::npos ? "" : key.substr(first, last - first + 1);
  for (auto & c : out) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (c == '-' || c == ' ') {c = '_';}
  }
  return out;
}

const ParameterDefinition & ParameterGroup::operator[](const std::string & key) const
{
  const auto normalized = normalize(key);
  const auto alias = aliases_.find(normalized);
  const auto & canonical = alias != aliases_.end() ? alias->second : normalized;
  const auto it = canonical_.find(canonical);
  if (it == canonical_.end()) {
    throw std::out_of_range("Unknown parameter '" + key + "'");
  }
  return it->second;
}

bool ParameterGroup::contains(const std::string & key) const
{
  const auto normalized = normalize(key);
  return aliases_.count(normalized) || canonical_.count(normalized);
}

}  // namespace sparkmax
