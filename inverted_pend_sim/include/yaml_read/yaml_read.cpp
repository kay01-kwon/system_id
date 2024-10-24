#include "yaml_read.hpp"

YAMLRead::YAMLRead(const string &file_path)
:file_path_(file_path)
{
    load_yaml_file();

}

void YAMLRead::get_inertial_params(InertialParams_t &inertial_params) const
{
    inertial_params = inertial_params_;
}

void YAMLRead::get_aero_coeffs(AeroCoeffs_t &aero_coeffs) const
{
    aero_coeffs = aero_coeffs_;
}

void YAMLRead::load_yaml_file()
{
    try{
        
        cout << "Loaded YAML file: " << file_path_ << endl;
        doc_ = YAML::LoadFile(file_path_);

        get_inertial_params_from_yaml(doc_);
        get_aero_coeffs_from_yaml(doc_);

    }
    catch(const YAML::BadFile &e)
    {
        cerr << "Error: " << e.what() << endl;
    }
}

void YAMLRead::get_inertial_params_from_yaml(const Node &config)
{
    auto inertial_params_config = config["inertial_params"];

    double Jxx, Jxy, Jxz, Jyy, Jyz, Jzz;
    double m, x_CM, y_CM, z_CM;

    assert(inertial_params_config > 0);

    assert(inertial_params_config["MOI"]["Jxx"] != 0);
    assert(inertial_params_config["MOI"]["Jxy"] != 0);
    assert(inertial_params_config["MOI"]["Jxz"] != 0);
    assert(inertial_params_config["MOI"]["Jyy"] != 0);
    assert(inertial_params_config["MOI"]["Jyz"] != 0);
    assert(inertial_params_config["MOI"]["Jzz"] != 0);

    assert(inertial_params_config["mass"]["m"] != 0);

    assert(inertial_params_config["CM"]["x"] != 0);
    assert(inertial_params_config["CM"]["y"] != 0);
    assert(inertial_params_config["CM"]["z"] != 0);

    Jxx = inertial_params_config["MOI"]["Jxx"].as<double>();
    Jxy = inertial_params_config["MOI"]["Jxy"].as<double>();
    Jxz = inertial_params_config["MOI"]["Jxz"].as<double>();
    Jyy = inertial_params_config["MOI"]["Jyy"].as<double>();
    Jyz = inertial_params_config["MOI"]["Jyz"].as<double>();
    Jzz = inertial_params_config["MOI"]["Jzz"].as<double>();

    m = inertial_params_config["mass"]["m"].as<double>();

    x_CM = inertial_params_config["CM"]["x"].as<double>();
    y_CM = inertial_params_config["CM"]["y"].as<double>();
    z_CM = inertial_params_config["CM"]["z"].as<double>();

    inertial_params_.J(0,0) = Jxx;
    inertial_params_.J(0,1) = Jxy;
    inertial_params_.J(0,2) = Jxz;

    inertial_params_.J(1,0) = Jxy;
    inertial_params_.J(1,1) = Jyy;
    inertial_params_.J(1,2) = Jyz;

    inertial_params_.J(2,0) = Jxz;
    inertial_params_.J(1,2) = Jyz;
    inertial_params_.J(2,2) = Jzz;

    inertial_params_.m = m;

    inertial_params_.CG_r_CM(0) = x_CM;
    inertial_params_.CG_r_CM(1) = y_CM;
    inertial_params_.CG_r_CM(2) = z_CM;

}

void YAMLRead::get_aero_coeffs_from_yaml(const Node &config)
{
    auto aero_coeffs_config = config["aero_coeffs"];

    double C_T, C_M, l;

    assert(aero_coeffs_config > 0);

    assert(aero_coeffs_config["C_T"] != 0);
    assert(aero_coeffs_config["C_M"] != 0);
    assert(aero_coeffs_config["l"] != 0);

    C_T = aero_coeffs_config["C_T"].as<double>();
    C_M = aero_coeffs_config["C_M"].as<double>();
    l = aero_coeffs_config["l"].as<double>();

    aero_coeffs_.C_T = C_T;
    aero_coeffs_.C_M = C_M;
    aero_coeffs_.l = l;

}