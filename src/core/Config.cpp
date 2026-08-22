#include "core/Config.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace tca {

bool Config::load(const std::string& path) {
    std::ifstream f(path);
    if(!f) return false;

    std::string currentSection;
    std::string line;
    while(std::getline(f, line)) {
        // Strip comments and whitespace
        auto commentPos = line.find('#');
        if(commentPos != std::string::npos) line = line.substr(0, commentPos);
        // Trim
        auto ltrim = [](std::string& s){ s.erase(s.begin(), std::find_if(s.begin(),s.end(),[](unsigned char c){return !std::isspace(c);})); };
        auto rtrim = [](std::string& s){ s.erase(std::find_if(s.rbegin(),s.rend(),[](unsigned char c){return !std::isspace(c);}).base(), s.end()); };
        ltrim(line); rtrim(line);
        if(line.empty()) continue;

        if(line.front()=='[' && line.back()==']') {
            currentSection = line.substr(1, line.size()-2);
            continue;
        }
        auto eq = line.find('=');
        if(eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq+1);
        ltrim(key); rtrim(key); ltrim(val); rtrim(val);
        if(!currentSection.empty()) key = currentSection + "." + key;
        data_[key] = val;
    }
    return true;
}

void Config::set(const std::string& k, const std::string& v){ data_[k]=v; }

std::string Config::getString(const std::string& k, const std::string& def) const {
    auto it = data_.find(k); return it!=data_.end() ? it->second : def;
}
double Config::getDouble(const std::string& k, double def) const {
    auto it = data_.find(k); if(it==data_.end()) return def;
    try{ return std::stod(it->second); } catch(...){ return def; }
}
int Config::getInt(const std::string& k, int def) const {
    auto it = data_.find(k); if(it==data_.end()) return def;
    try{ return std::stoi(it->second); } catch(...){ return def; }
}
bool Config::getBool(const std::string& k, bool def) const {
    auto s = getString(k, def?"true":"false");
    return (s=="true"||s=="1"||s=="yes");
}
void Config::dump() const {
    for(auto& [k,v]: data_) std::cout << k << " = " << v << "\n";
}

} // namespace tca
