#include "sparkmax_protocol/json.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace sparkmax_protocol::json
{

// -----------------------------------------------------------------------------
// Value
// -----------------------------------------------------------------------------

Value::Value() = default;

Value Value::makeBool(bool value)
{
  Value v;
  v.type_ = Type::Bool;
  v.bool_ = value;
  return v;
}

Value Value::makeString(std::string value)
{
  Value v;
  v.type_ = Type::String;
  v.string_ = std::move(value);
  return v;
}

Value Value::makeArray(Array value)
{
  Value v;
  v.type_ = Type::Array;
  v.array_ = std::make_shared<Array>(std::move(value));
  return v;
}

Value Value::makeObject(Object value)
{
  Value v;
  v.type_ = Type::Object;
  v.object_ = std::make_shared<Object>(std::move(value));
  return v;
}

Value Value::makeDouble(double value)
{
  Value v;
  v.type_ = Type::Number;
  v.number_ = value;
  return v;
}

Value Value::makeInteger(int64_t value)
{
  Value v;
  v.type_ = Type::Number;
  v.integer_ = true;
  v.negative_ = value < 0;
  v.magnitude_ = v.negative_ ?
    static_cast<uint64_t>(-(value + 1)) + 1u : static_cast<uint64_t>(value);
  v.number_ = static_cast<double>(value);
  return v;
}

Value Value::makeUnsigned(uint64_t value)
{
  Value v;
  v.type_ = Type::Number;
  v.integer_ = true;
  v.magnitude_ = value;
  v.number_ = static_cast<double>(value);
  return v;
}

namespace
{
const char * typeName(Value::Type type)
{
  switch (type) {
    case Value::Type::Null: return "null";
    case Value::Type::Bool: return "bool";
    case Value::Type::Number: return "number";
    case Value::Type::String: return "string";
    case Value::Type::Array: return "array";
    case Value::Type::Object: return "object";
  }
  return "?";
}

[[noreturn]] void wrongType(Value::Type actual, const char * expected)
{
  throw TypeError(std::string("JSON: se esperaba ") + expected + ", se encontró " +
          typeName(actual));
}
}  // namespace

bool Value::asBool() const
{
  if (type_ != Type::Bool) {wrongType(type_, "bool");}
  return bool_;
}

double Value::asDouble() const
{
  if (type_ != Type::Number) {wrongType(type_, "number");}
  return number_;
}

int64_t Value::asInt64() const
{
  if (type_ != Type::Number) {wrongType(type_, "number");}
  if (!integer_) {
    if (!std::isfinite(number_) || std::trunc(number_) != number_) {
      throw TypeError("JSON: el número no es entero");
    }
    return static_cast<int64_t>(number_);
  }
  constexpr uint64_t max_pos = static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
  if (negative_) {
    if (magnitude_ > max_pos + 1u) {throw TypeError("JSON: entero fuera de rango int64");}
    return magnitude_ == max_pos + 1u ? std::numeric_limits<int64_t>::min() :
           -static_cast<int64_t>(magnitude_);
  }
  if (magnitude_ > max_pos) {throw TypeError("JSON: entero fuera de rango int64");}
  return static_cast<int64_t>(magnitude_);
}

uint64_t Value::asUint64() const
{
  if (type_ != Type::Number) {wrongType(type_, "number");}
  if (!integer_) {
    if (!std::isfinite(number_) || std::trunc(number_) != number_ || number_ < 0) {
      throw TypeError("JSON: el número no es un entero no negativo");
    }
    return static_cast<uint64_t>(number_);
  }
  if (negative_ && magnitude_ != 0) {throw TypeError("JSON: entero negativo en uint64");}
  return magnitude_;
}

const std::string & Value::asString() const
{
  if (type_ != Type::String) {wrongType(type_, "string");}
  return string_;
}

const Value::Array & Value::asArray() const
{
  if (type_ != Type::Array) {wrongType(type_, "array");}
  return *array_;
}

const Value::Object & Value::asObject() const
{
  if (type_ != Type::Object) {wrongType(type_, "object");}
  return *object_;
}

const Value * Value::find(std::string_view key) const
{
  if (type_ != Type::Object) {return nullptr;}
  for (const auto & member : *object_) {
    if (member.key == key) {return &member.value;}
  }
  return nullptr;
}

const Value & Value::at(std::string_view key) const
{
  const Value * value = find(key);
  if (value == nullptr) {
    throw TypeError("JSON: falta la clave '" + std::string(key) + "'");
  }
  return *value;
}

std::string Value::stringOr(std::string_view key, std::string fallback) const
{
  const Value * value = find(key);
  return value != nullptr && value->isString() ? value->string_ : std::move(fallback);
}

double Value::doubleOr(std::string_view key, double fallback) const
{
  const Value * value = find(key);
  return value != nullptr && value->isNumber() ? value->number_ : fallback;
}

bool Value::boolOr(std::string_view key, bool fallback) const
{
  const Value * value = find(key);
  return value != nullptr && value->isBool() ? value->bool_ : fallback;
}

// -----------------------------------------------------------------------------
// Parser recursivo descendente
// -----------------------------------------------------------------------------

namespace
{

class Parser
{
public:
  explicit Parser(std::string_view text)
  : text_(text) {}

  Value parseDocument()
  {
    skipWhitespace();
    Value value = parseValue(0);
    skipWhitespace();
    if (pos_ != text_.size()) {fail("contenido extra después del documento");}
    return value;
  }

private:
  static constexpr int kMaxDepth = 256;

  [[noreturn]] void fail(const std::string & message) const
  {
    std::size_t line = 1;
    std::size_t column = 1;
    for (std::size_t i = 0; i < pos_ && i < text_.size(); ++i) {
      if (text_[i] == '\n') {
        ++line;
        column = 1;
      } else {
        ++column;
      }
    }
    std::ostringstream out;
    out << "JSON inválido (línea " << line << ", columna " << column << "): " << message;
    throw ParseError(out.str());
  }

  void skipWhitespace()
  {
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++pos_;
      } else {
        break;
      }
    }
  }

  char peek() const {return pos_ < text_.size() ? text_[pos_] : '\0';}

  void expect(char c)
  {
    if (peek() != c) {fail(std::string("se esperaba '") + c + "'");}
    ++pos_;
  }

  void expectLiteral(std::string_view literal)
  {
    if (text_.substr(pos_, literal.size()) != literal) {
      fail("literal inválido, se esperaba '" + std::string(literal) + "'");
    }
    pos_ += literal.size();
  }

  Value parseValue(int depth)
  {
    if (depth > kMaxDepth) {fail("anidamiento demasiado profundo");}
    switch (peek()) {
      case '{': return parseObject(depth);
      case '[': return parseArray(depth);
      case '"': return Value::makeString(parseString());
      case 't': expectLiteral("true"); return Value::makeBool(true);
      case 'f': expectLiteral("false"); return Value::makeBool(false);
      case 'n': expectLiteral("null"); return Value();
      default: return parseNumber();
    }
  }

  Value parseObject(int depth)
  {
    expect('{');
    Value::Object members;
    skipWhitespace();
    if (peek() == '}') {
      ++pos_;
      return Value::makeObject(std::move(members));
    }
    while (true) {
      skipWhitespace();
      if (peek() != '"') {fail("se esperaba una clave de texto");}
      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();
      members.push_back(Member{std::move(key), parseValue(depth + 1)});
      skipWhitespace();
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      expect('}');
      break;
    }
    return Value::makeObject(std::move(members));
  }

  Value parseArray(int depth)
  {
    expect('[');
    Value::Array items;
    skipWhitespace();
    if (peek() == ']') {
      ++pos_;
      return Value::makeArray(std::move(items));
    }
    while (true) {
      skipWhitespace();
      items.push_back(parseValue(depth + 1));
      skipWhitespace();
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      expect(']');
      break;
    }
    return Value::makeArray(std::move(items));
  }

  unsigned parseHex4()
  {
    if (pos_ + 4 > text_.size()) {fail("escape \\u incompleto");}
    unsigned value = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = text_[pos_++];
      value <<= 4;
      if (c >= '0' && c <= '9') {
        value |= static_cast<unsigned>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        value |= static_cast<unsigned>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        value |= static_cast<unsigned>(c - 'A' + 10);
      } else {
        fail("dígito hexadecimal inválido en \\u");
      }
    }
    return value;
  }

  static void appendUtf8(std::string & out, unsigned cp)
  {
    if (cp < 0x80) {
      out.push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
      out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
      out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
      out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
  }

  std::string parseString()
  {
    expect('"');
    std::string out;
    while (true) {
      if (pos_ >= text_.size()) {fail("texto sin cerrar");}
      const char c = text_[pos_++];
      if (c == '"') {break;}
      if (static_cast<unsigned char>(c) < 0x20) {fail("carácter de control en texto");}
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (pos_ >= text_.size()) {fail("escape incompleto");}
      const char e = text_[pos_++];
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
            unsigned cp = parseHex4();
            if (cp >= 0xD800 && cp <= 0xDBFF) {
              if (text_.substr(pos_, 2) != "\\u") {fail("par sustituto incompleto");}
              pos_ += 2;
              const unsigned low = parseHex4();
              if (low < 0xDC00 || low > 0xDFFF) {fail("par sustituto inválido");}
              cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
            } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
              fail("sustituto bajo sin par");
            }
            appendUtf8(out, cp);
            break;
          }
        default: fail("escape desconocido");
      }
    }
    return out;
  }

  Value parseNumber()
  {
    const std::size_t start = pos_;
    bool is_integer = true;
    if (peek() == '-') {++pos_;}
    if (peek() == '0') {
      ++pos_;
    } else if (peek() >= '1' && peek() <= '9') {
      while (peek() >= '0' && peek() <= '9') {++pos_;}
    } else {
      fail("valor inesperado");
    }
    if (peek() == '.') {
      is_integer = false;
      ++pos_;
      if (!(peek() >= '0' && peek() <= '9')) {fail("se esperaban dígitos decimales");}
      while (peek() >= '0' && peek() <= '9') {++pos_;}
    }
    if (peek() == 'e' || peek() == 'E') {
      is_integer = false;
      ++pos_;
      if (peek() == '+' || peek() == '-') {++pos_;}
      if (!(peek() >= '0' && peek() <= '9')) {fail("exponente inválido");}
      while (peek() >= '0' && peek() <= '9') {++pos_;}
    }
    const char * first = text_.data() + start;
    const char * last = text_.data() + pos_;

    if (is_integer) {
      if (*first == '-') {
        int64_t value = 0;
        const auto result = std::from_chars(first, last, value);
        if (result.ec == std::errc() && result.ptr == last) {return Value::makeInteger(value);}
      } else {
        uint64_t value = 0;
        const auto result = std::from_chars(first, last, value);
        if (result.ec == std::errc() && result.ptr == last) {return Value::makeUnsigned(value);}
      }
      // Entero demasiado grande: se conserva como double.
    }
    // from_chars no depende del locale (strtod sí: "0,5" vs "0.5").
    double value = 0.0;
    const auto result = std::from_chars(first, last, value);
    if (result.ptr != last ||
      (result.ec != std::errc() && result.ec != std::errc::result_out_of_range))
    {
      fail("número inválido");
    }
    return Value::makeDouble(value);
  }

  std::string_view text_;
  std::size_t pos_{0};
};

}  // namespace

Value parse(std::string_view text)
{
  return Parser(text).parseDocument();
}

Value parseFile(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("No se pudo abrir el archivo JSON: " + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  try {
    return parse(buffer.str());
  } catch (const ParseError & error) {
    throw ParseError(path + ": " + error.what());
  }
}

}  // namespace sparkmax_protocol::json
