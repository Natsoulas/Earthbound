/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef UTILITIES_CSV_WRITER_HPP
#define UTILITIES_CSV_WRITER_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <sstream>

namespace utilities {

class CSVWriter {
public:
    CSVWriter(const std::string& filename, const std::vector<std::string>& headers) 
        : filename_(filename) {
        // Create directory if it doesn't exist
        std::filesystem::create_directories(std::filesystem::path(filename).parent_path());
        
        // Open file and write headers
        file_.open(filename);
        file_ << std::setprecision(16);
        
        // Write headers
        for (size_t i = 0; i < headers.size(); ++i) {
            file_ << headers[i];
            if (i < headers.size() - 1) file_ << ",";
        }
        file_ << "\n";
    }

    ~CSVWriter() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    template<typename... Args>
    void writeRow(const Args&... args) {
        writeValues(args...);
        file_ << "\n";
    }

private:
    std::string filename_;
    std::ofstream file_;

    // Base case for variadic template
    template<typename T>
    void writeValues(const T& value) {
        file_ << value;
    }

    // Recursive case for variadic template
    template<typename T, typename... Args>
    void writeValues(const T& value, const Args&... args) {
        file_ << value << ",";
        writeValues(args...);
    }
};

} // namespace utilities

#endif // UTILITIES_CSV_WRITER_HPP