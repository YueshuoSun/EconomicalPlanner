#include "my_env.h"

bool MyEnvironment::Read(const std::string &env_file) {
    std::ifstream is(env_file, std::ios::binary);
    if (is.is_open()) {
        json j = json::parse(is);
        *this = j;

        return true;
    } else {
        return false;
    }
}

void MyEnvironment::Save(const std::string &env_file) const {
    std::ofstream os(env_file, std::ios::binary);
    if (os.is_open()) {
        json j = *this;
        os << j;
    }
}
