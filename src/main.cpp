
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void print_usage(const string &program_name)
{
  cerr << "Usage: " << program_name << " [OPTION]... INPUT OUTPUT" << endl
       << endl
       << "  INPUT     path to input file." << endl
       << "  OUTPUT    path to output file." << endl
       << endl
       << "Options:" << endl
       << "  -lidar={yes|no}       Do|don't use LIDAR sensor." << endl
       << "  -radar={yes|no}       Do|don't use RADAR sensor." << endl
       << "  -std_a=FLOAT          Stdev for the longitudinal acceleration in m/s^2." << endl
       << "  -std_yawdd=FLOAT      Stdev for the the yaw acceleration in rad/s^2." << endl
       << "  -help, --help         Print this message." << endl
       << endl
  ;
}

void exit_with_usage(const string &program_name, int exit_code)
{
  print_usage(program_name);
  exit(exit_code);
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  string in_file_name_;
  string out_file_name_;
  bool use_laser_ = true;
  bool use_radar_ = true;
  float std_a_ = -1;
  float std_yawdd_ = -1;
  
  // Parse arguments
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == '-') {
      if (!strcmp(argv[i], "-help") || !strcmp(argv[i], "--help"))
        exit_with_usage(argv[0], 0);
      else if (!strcmp(argv[i], "-lidar=no"))
        use_laser_ = false;
      else if (!strcmp(argv[i], "-lidar=yes"))
        use_laser_ = true;
      else if (!strcmp(argv[i], "-radar=no"))
        use_radar_ = false;
      else if (!strcmp(argv[i], "-radar=yes"))
        use_radar_ = true;
      else if (!strncmp(argv[i], "-std_a=", 7)) {
        std_a_ = strtof(argv[i]+7, nullptr);
        if (!std::isfinite(std_a_) || std_a_ < 0) {
          cerr << "Invalid value for std_a" << endl;
          exit(EXIT_FAILURE);
        }
      }
      else if (!strncmp(argv[i], "-std_yawdd=", 11)) {
        std_yawdd_ = strtof(argv[i]+11, nullptr);
        if (!std::isfinite(std_yawdd_) || std_yawdd_ < 0) {
          cerr << "Invalid value for std_yawdd" << endl;
          exit(EXIT_FAILURE);
        }
      }
      else {
        cerr << "Unknown option: " << argv[i] << endl;
        exit_with_usage(argv[0], EXIT_FAILURE);
      }
    } else {
      if (in_file_name_.empty())
        in_file_name_ = argv[i];
      else if (out_file_name_.empty())
        out_file_name_ = argv[i];
      else {
        cerr << "Too many arguments." << endl;
        exit_with_usage(argv[0], EXIT_FAILURE);
      }
    }
  }
  
  if (in_file_name_.empty() || out_file_name_.empty()) {
    cerr << "Input and/or output file missing." << endl;
    exit(EXIT_FAILURE);
  }
  
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // laser measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // radar measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

      // read ground truth data to compare later
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      gt_package.gt_values_ = VectorXd(4);
      gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_package);
  }

  // Create a UKF instance
  UKF ukf;
  ukf.use_laser_ = use_laser_;
  ukf.use_radar_ = use_radar_;
  
  if (std_a_ > 0)
    ukf.std_a_ = std_a_;
  
  if (std_yawdd_ > 0)
    ukf.std_yawdd_ = std_yawdd_;
  
  cerr << "UKF:"
       << " [ LIDAR " << (ukf.use_laser_ ? "ON" : "OFF")
       << " | RADAR " << (ukf.use_radar_ ? "ON" : "OFF")
       << " | std_a = " << ukf.std_a_
       << " | std_yawdd = " << ukf.std_yawdd_
       << " ]" << endl;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // start filtering from the second frame (the speed is unknown in the first
  // frame)

  size_t number_of_measurements = measurement_pack_list.size();

  // column names for output file
  out_file_ << "time_stamp" << "\t";  
  out_file_ << "px_state" << "\t";
  out_file_ << "py_state" << "\t";
  out_file_ << "v_state" << "\t";
  out_file_ << "yaw_angle_state" << "\t";
  out_file_ << "yaw_rate_state" << "\t";
  out_file_ << "sensor_type" << "\t";
  out_file_ << "NIS" << "\t";  
  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";
  out_file_ << "px_ground_truth" << "\t";
  out_file_ << "py_ground_truth" << "\t";
  out_file_ << "vx_ground_truth" << "\t";
  out_file_ << "vy_ground_truth" << "\n";


  for (size_t k = 0; k < number_of_measurements; ++k) {
    // Call the UKF-based fusion
    if (!ukf.ProcessMeasurement(measurement_pack_list[k]))
      continue;

    // timestamp
    out_file_ << measurement_pack_list[k].timestamp_ << "\t"; // pos1 - est

    // output the state vector
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // output lidar and radar specific data
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // sensor type
      out_file_ << "lidar" << "\t";

      // NIS value
      out_file_ << ukf.NIS_laser_ << "\t";

      // output the lidar sensor measurement px and py
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";

    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // sensor type
      out_file_ << "radar" << "\t";

      // NIS value
      out_file_ << ukf.NIS_radar_ << "\t";

      // output radar measurement in cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // px measurement
      out_file_ << ro * sin(phi) << "\t"; // py measurement
    }

    // output the ground truth
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    // convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);

    float x_estimate_ = ukf.x_(0);
    float y_estimate_ = ukf.x_(1);
    float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
    float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
    
    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
    
    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);

  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "RMSE: " << tools.CalculateRMSE(estimations, ground_truth).transpose() << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  //cout << "Done!" << endl;
  return 0;
}
