#include "yaml_read.hpp"

YAMLRead::YAMLRead(const string &file_path)
:file_path_(file_path)
{

}

void YAMLRead::load_yaml_file()
{
    try{
        
        cout << "Loaded YAML file: " << file_path_ << endl;
        doc_ = YAML::LoadFile(file_path_);

    }
    catch(const YAML::BadFile &e)
    {
        cerr << "Error: " << e.what() << endl;
    }
}

void YAMLRead::read_inertial_params(InertialParams_t &inertial_params) const
{
    inertial_params = inertial_params_;
}

void YAMLRead::read_aero_coeffs(AeroCoeffs_t &aero_coeffs) const
{
    aero_coeffs = aero_coeffs_;
}

void YAMLRead::get_inertial_params_from_yaml(const Node &config)
{
}

void YAMLRead::get_aero_coeffs_from_yaml(const Node &config)
{
}
