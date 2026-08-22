#pragma once
#include <string>
#include <unordered_map>

namespace tca {

/// Flat key-value configuration loaded from a .cfg file.
/// Keys: "section.key", values: strings converted on demand.
class Config {
public:
    Config() = default;

    /// Load from file; returns false on failure.
    bool load(const std::string& path);

    /// Store a key/value pair programmatically.
    void set(const std::string& key, const std::string& value);

    std::string  getString (const std::string& key, const std::string& def = "") const;
    double       getDouble (const std::string& key, double def = 0.0)            const;
    int          getInt    (const std::string& key, int    def = 0)              const;
    bool         getBool   (const std::string& key, bool   def = false)          const;

    void dump() const;

private:
    std::unordered_map<std::string, std::string> data_;
};

} // namespace tca
