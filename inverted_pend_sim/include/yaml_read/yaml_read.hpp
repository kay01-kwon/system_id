#ifndef YAML_READ_HPP
#define YAML_READ_HPP
#include "definitions/simulation_param_def.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

using std::string;
using std::cerr;
using YAML::Node;

class YAMLRead
{
    public:

    YAMLRead(const string &file_path);
    ~YAMLRead() = default;

    void get_inertial_params(InertialParams_t &inertial_params) const;

    void get_aero_coeffs(AeroCoeffs_t &aero_coeffs) const;

    private:

    void load_yaml_file();

    void get_inertial_params_from_yaml(const Node &config);
    
    void get_aero_coeffs_from_yaml(const Node &config);

    string file_path_;
    Node doc_;

    InertialParams_t inertial_params_;
    AeroCoeffs_t aero_coeffs_;
};


#endif // YAML_READ_HPP