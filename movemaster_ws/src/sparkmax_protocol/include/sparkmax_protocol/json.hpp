// json.hpp
//
// Parser JSON mínimo y sin dependencias externas, suficiente para leer la
// especificación de tramas de REV (spark-frames-*.json). Conserva el orden de
// las claves de los objetos (igual que el diccionario de Python) y representa
// los enteros sin pérdida de precisión hasta 64 bits.

#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sparkmax_protocol::json
{

class ParseError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class TypeError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

struct Member;

class Value
{
public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  using Array = std::vector<Value>;
  using Object = std::vector<Member>;

  Value();
  static Value makeBool(bool value);
  static Value makeString(std::string value);
  static Value makeArray(Array value);
  static Value makeObject(Object value);
  static Value makeDouble(double value);
  static Value makeInteger(int64_t value);
  static Value makeUnsigned(uint64_t value);

  Type type() const {return type_;}
  bool isNull() const {return type_ == Type::Null;}
  bool isBool() const {return type_ == Type::Bool;}
  bool isNumber() const {return type_ == Type::Number;}
  bool isString() const {return type_ == Type::String;}
  bool isArray() const {return type_ == Type::Array;}
  bool isObject() const {return type_ == Type::Object;}
  /// true si el número se escribió sin parte decimal ni exponente.
  bool isInteger() const {return type_ == Type::Number && integer_;}

  bool asBool() const;
  double asDouble() const;
  int64_t asInt64() const;
  uint64_t asUint64() const;
  const std::string & asString() const;
  const Array & asArray() const;
  const Object & asObject() const;

  /// Busca una clave en un objeto; nullptr si no existe o no es objeto.
  const Value * find(std::string_view key) const;
  /// Igual que find() pero lanza TypeError si no existe.
  const Value & at(std::string_view key) const;
  bool contains(std::string_view key) const {return find(key) != nullptr;}

  std::string stringOr(std::string_view key, std::string fallback) const;
  double doubleOr(std::string_view key, double fallback) const;
  bool boolOr(std::string_view key, bool fallback) const;

private:
  Type type_{Type::Null};
  bool bool_{false};
  bool integer_{false};
  bool negative_{false};
  double number_{0.0};
  uint64_t magnitude_{0};  // valor absoluto exacto si integer_
  std::string string_;
  std::shared_ptr<Array> array_;
  std::shared_ptr<Object> object_;
};

struct Member
{
  std::string key;
  Value value;
};

/// Analiza un documento JSON completo.
Value parse(std::string_view text);

/// Lee y analiza un archivo JSON.
Value parseFile(const std::string & path);

}  // namespace sparkmax_protocol::json
