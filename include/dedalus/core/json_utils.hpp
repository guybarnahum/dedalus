#pragma once

#include <cstddef>
#include <string>

namespace dedalus {

// Returns a copy of value with JSON special characters backslash-escaped.
// Does not add surrounding quotes.
inline std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"':  escaped += "\\\""; break;
            case '\n': escaped += "\\n";  break;
            case '\r': escaped += "\\r";  break;
            case '\t': escaped += "\\t";  break;
            default:   escaped.push_back(ch); break;
        }
    }
    return escaped;
}

// Returns the value JSON-escaped and wrapped in double quotes.
inline std::string q(const std::string& value) {
    return "\"" + json_escape(value) + "\"";
}

// Accumulates comma-prefixed JSON key:value pairs for building event /
// telemetry JSON fragments.  The caller writes the leading field (e.g.
// "\"event\":\"foo\""); all further fields are appended via kv().
// Double/int/size_t output uses std::to_string; bool emits literal
// "true"/"false"; strings are passed through q().
class JsonFields {
    std::string buf_;
public:
    JsonFields& kv(const char* key, const std::string& val) {
        buf_ += ",\""; buf_ += key; buf_ += "\":"; buf_ += q(val); return *this;
    }
    JsonFields& kv(const char* key, double val) {
        buf_ += ",\""; buf_ += key; buf_ += "\":"; buf_ += std::to_string(val); return *this;
    }
    JsonFields& kv(const char* key, int val) {
        buf_ += ",\""; buf_ += key; buf_ += "\":"; buf_ += std::to_string(val); return *this;
    }
    JsonFields& kv(const char* key, std::size_t val) {
        buf_ += ",\""; buf_ += key; buf_ += "\":"; buf_ += std::to_string(val); return *this;
    }
    JsonFields& kv(const char* key, bool val) {
        buf_ += ",\""; buf_ += key; buf_ += "\":"; buf_ += val ? "true" : "false"; return *this;
    }
    [[nodiscard]] std::string str() const { return buf_; }
};

}  // namespace dedalus
