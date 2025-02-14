#include <math.h>

#include <fstream>
#include <iostream>

using namespace std;

class SpringMassDamperSystem {
 private:
  const float mass_;            // mass [kg]
  const float dampingFactor_;   // damping factor [Ns/m]
  const float springConstant_;  // spring constant [N/m]
  const float dt_;              // sampling time [s]

  float position_ = 0.0;
  float velocity_ = 0.0;
  float acceleration_ = 0.0;

 public:
  SpringMassDamperSystem(float m, float c, float k, float dt)
      : mass_(m), springConstant_(k), dampingFactor_(c), dt_(dt) {}

  void update(float force) {
    acceleration_ =
        (force - dampingFactor_ * velocity_ - springConstant_ * position_) /
        mass_;
    velocity_ += acceleration_ * dt_;
    position_ += velocity_ * dt_;
  }

  float getPosition() const { return position_; }
  float getVelocity() const { return velocity_; }
};

class PseudoDifferential {
 private:
  const float g_;   // cut-off frequency [Hz]
  const float dt_;  // sampling time [s]

  float x_lpf_ = 0.0;

 public:
  PseudoDifferential(float g, float dt) : g_(g), dt_(dt) {}

  double update(float x) {
    x_lpf_ = (x_lpf_ + g_ * dt_ * x) / (1 + g_ * dt_);
    float y = g_ * (x - x_lpf_);
    return y;
  }
};

class DisturbanceObserver {
 private:
  const float M_n_;  // nominal mass [kg]
  const float g_;    // cut-off frequency [Hz]
  const float dt_;   // sampling time [s]

  float tau_dis_hat_ = 0.0;
  float tmp_ = 0.0;
  float tmp_lpf_ = 0.0;

 public:
  DisturbanceObserver(float M_n, float g, float dt)
      : M_n_(M_n), g_(g), dt_(dt) {}

  float update(float tau_ref, float vel_res) {
    tmp_ = tau_ref + M_n_ * g_ * vel_res;
    tmp_lpf_ = (tmp_lpf_ + g_ * dt_ * tmp_) / (1 + g_ * dt_);
    tau_dis_hat_ = tmp_lpf_ - M_n_ * g_ * vel_res;
    return tau_dis_hat_;
  };
};

class Controller {
 private:
  const float Kp_;
  const float Kd_;
  const float dt_;  // sampling time [s]
  PseudoDifferential pseudo_differential_;
  DisturbanceObserver disturbance_observer_;

  float pos_res_ = 0.0;
  float vel_res_ = 0.0;
  float tau_dis_hat_ = 0.0;
  float tau_ref_ = 0.0;

 public:
  Controller(float M_hat, float Kp, float Kd, float g, float dt)
      : Kp_(Kp),
        Kd_(Kd),
        dt_(dt),
        pseudo_differential_(g, dt),
        disturbance_observer_(M_hat, g, dt) {}

  float update(float pos_cmd, float pos_res) {
    vel_res_ = pseudo_differential_.update(pos_res);
    tau_dis_hat_ = disturbance_observer_.update(tau_ref_, vel_res_);
    // tau_ref = Kp * (pos_cmd - pos_res);
    // tau_ref = Kp * (pos_cmd - pos_res) + Kd * vel_res;
    // tau_ref = Kp * (pos_cmd - pos_res) + tau_dis_hat;
    tau_ref_ = Kp_ * (pos_cmd - pos_res) + Kd_ * (0 - vel_res_) + tau_dis_hat_;
    return tau_ref_;
  }

  float getPosition() const { return pos_res_; }
  float getVelocity() const { return vel_res_; }
  float getDisturbance() const { return tau_dis_hat_; }
};

int main() {
  const float dt = 0.002;      // sampling time [s]
  const float sim_time = 5.0;  // simulation time [s]

  const float M = 1.0;   // mass [kg]
  const float C = 10.0;  // damping factor [Ns/m]
  const float K = 0.0;   // spring constant [N/m]
  SpringMassDamperSystem system(M, C, K, dt);

  const float M_n = M * 1.5;           // nominal mass [kg]
  const float pole = 10;         // pole
  const float Kp = pole * pole;  // p gain
  const float Kd = 2 * pole;     // d gain
  const float g = 30.0;          // cut-off frequency [Hz]
  Controller controller(M_n, Kp, Kd, g, dt);

  float pos_cmd = 1.0;  // position control command [m]
  float tau_dis = 1.0;  // disturbance input [N]

  // output csv file
  string output_csv_file_path = "output.csv";
  ofstream output_csv(output_csv_file_path);
  output_csv << "time,"
                "pos_cmd,"
                "tau_ref,"
                "pos_res,"
                "vel_res,"
                "vel_res_hat,"
                "tau_dis,"
                "tau_dis_hat"
             << std::endl;

  for (int i = 0; i < int(sim_time / dt); i++) {
    // observation
    float pos_res = system.getPosition();

    // pos_cmd = 1.0 + 0.5 * sin(2 * M_PI * 0.5 * i * dt);
    // pos_cmd = 0.5 * sin(2 * M_PI * 0.5 * i * dt);

    // calculate control input
    float tau_ref = controller.update(pos_cmd, pos_res);

    // print
    float vel_res = system.getVelocity();
    float vel_res_hat = controller.getVelocity();
    float tau_dis_hat = controller.getDisturbance();
    string row = to_string(i * dt) + "," + to_string(pos_cmd) + "," +
                 to_string(tau_ref) + "," + to_string(pos_res) + "," +
                 to_string(vel_res) + "," + to_string(vel_res_hat) + "," +
                 to_string(tau_dis) + "," + to_string(tau_dis_hat);
    cout << row << endl;
    output_csv << row << endl;

    // update
    system.update(tau_ref - tau_dis);
  }
}
