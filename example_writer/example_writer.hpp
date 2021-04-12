#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pips {

class ExampleWriter {

public:

    ExampleWriter(const std::string& path)
    {
        f_.open(path);
        f_ << "[\n";
    }

    void InitStep()
    {
        if (in_step_)
            throw std::runtime_error("already in step");
        if (wrote_first_step_)
            f_ << ",\n";
        f_ << "  {\n";
        in_step_ = true;
        wrote_first_data_ = false;
    }

    void AddNumber(const std::string& name, const int dims[], double value)
    {
        const std::string d0 = std::to_string(dims[0]);
        const std::string d1 = std::to_string(dims[1]);
        const std::string d2 = std::to_string(dims[2]);

        if (wrote_first_data_)
            f_ << ",\n";
        f_ << "    \"" + name + "\": {\n";
        f_ << "      \"dim\": [" + d0 + ", " + d1 + ", " + d2 + "],\n";
        f_ << "      \"type\": \"NUM\",\n";
        f_ << "      \"name\": \"" + name + "\",\n";
        f_ << "      \"value\": " + std::to_string(value) + "\n";
        f_ << "    }";
        wrote_first_data_ = true;
    }

    void AddState(const std::string& name, const std::string& value)
    {
        if (wrote_first_data_)
            f_ << ",\n";
        f_ << "    \"" + name + "\": {\n";
        f_ << "      \"dim\": [0, 0, 0],\n";
        f_ << "      \"type\": \"STATE\",\n";
        f_ << "      \"name\": \"" + name + "\",\n";
        f_ << "      \"value\": \"" + value + "\"\n";
        f_ << "    }";
        wrote_first_data_ = true;
    }

    void CloseStep()
    {
        if (!in_step_)
            throw std::runtime_error("not currently in step");
        f_ << "\n  }";
        in_step_ = false;
        wrote_first_data_ = false;
        wrote_first_step_ = true;
    }

    ~ExampleWriter()
    {
        f_ << "\n]\n";
        f_.close();
    }

private:

    std::ofstream f_;
    bool in_step_ = false;
    bool wrote_first_step_ = false;
    bool wrote_first_data_ = false;

};

}; // namespace pips
