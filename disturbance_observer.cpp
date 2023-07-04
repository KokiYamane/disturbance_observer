#include <fstream>
#include <iostream>

using namespace std;

class SpringMassDamperSystem {
 private:
  const float mass;            // mass [kg]
  const float dampingFactor;   // damping factor [Ns/m]
  const float springConstant;  // spring constant [N/m]
  const float dt;              // sampling time [s]

  float position;
  float velocity;
  float acceleration;

 public:
  SpringMassDamperSystem(float m, float c, float k, float dt)
      : mass(m),
        springConstant(k),
        dampingFactor(c),
        dt(dt),
        position(0.0),
        velocity(0.0),
        acceleration(0.0) {}

  void update(float force) {
    acceleration =
        (force - dampingFactor * velocity - springConstant * position) / mass;
    velocity += acceleration * dt;
    position += velocity * dt;
  }

  float getPosition() const { return position; }
};

class PseudoDifferential {
 private:
  const float g;   // cut-off frequency [Hz]
  const float dt;  // sampling time [s]

  float x_lpf;

 public:
  PseudoDifferential(float g, float dt) : g(g), dt(dt), x_lpf(0.0) {}

  double update(float x) {
    x_lpf = (x_lpf + g * dt * x) / (1 + g * dt);
    float y = g * (x - x_lpf);
    return y;
  }
};

class DisturbanceObserver {
 private:
  const float M_n;  // nominal mass [kg]
  const float g;    // cut-off frequency [Hz]
  const float dt;   // sampling time [s]

  float tau_dis_hat;
  float tmp;
  float tmp_lpf;

 public:
  DisturbanceObserver(float M_n, float g, float dt)
      : M_n(M_n), g(g), dt(dt), tau_dis_hat(0.0), tmp(0.0), tmp_lpf(0.0) {}

  float update(float tau_ref, float vel_res) {
    tmp = tau_ref + M_n * g * vel_res;
    tmp_lpf = (tmp_lpf + g * dt * tmp) / (1 + g * dt);
    tau_dis_hat = tmp_lpf - M_n * g * vel_res;
    return tau_dis_hat;
  };
};

class Controller {
 private:
  const float Kp;
  const float Kd;
  const float dt;  // sampling time [s]
  PseudoDifferential pseudo_differential;
  DisturbanceObserver disturbance_observer;

  float pos_res;
  float vel_res;
  float tau_dis_hat;
  float tau_ref;

 public:
  Controller(float M_hat, float Kp, float Kd, float g, float dt)
      : Kp(Kp),
        Kd(Kd),
        dt(dt),
        pseudo_differential(g, dt),
        disturbance_observer(M_hat, g, dt),
        pos_res(0.0),
        vel_res(0.0),
        tau_dis_hat(0.0) {}

  float update(float pos_cmd, float pos_res) {
    vel_res = pseudo_differential.update(pos_res);
    tau_dis_hat = disturbance_observer.update(tau_ref, vel_res);
    tau_ref = Kp * (pos_cmd - pos_res);
    // tau_ref = Kp * (pos_cmd - pos_res) + Kd * vel_res;
    // tau_ref = Kp * (pos_cmd - pos_res) + tau_dis_hat;
    return tau_ref;
  }

  float getPosition() const { return pos_res; }
  float getVelocity() const { return vel_res; }
  float getDisturbance() const { return tau_dis_hat; }
};

int main() {
  const float dt = 0.002;       // sampling time [s]
  const float sim_time = 10.0;  // simulation time [s]

  const float M = 1.0;   // mass [kg]
  const float C = 10.0;  // damping factor [Ns/m]
  const float K = 0.0;   // spring constant [N/m]
  SpringMassDamperSystem system(M, C, K, dt);

  const float M_n = M;    // nominal mass [kg]
  const float Kp = 10.0;  // p gain
  const float Kd = 1.0;   // d gain
  const float g = 10.0;   // cut-off frequency [Hz]
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
                "tau_dis,"
                "tau_dis_hat"
             << std::endl;

  for (int i = 0; i < int(sim_time / dt); i++) {
    // observation
    float pos_res = system.getPosition();

    // calculate control input
    float tau_ref = controller.update(pos_cmd, pos_res);

    // print
    float vel_res = controller.getVelocity();
    float tau_dis_hat = controller.getDisturbance();
    string row = to_string(i * dt) + "," + to_string(pos_cmd) + "," +
                 to_string(tau_ref) + "," + to_string(pos_res) + "," +
                 to_string(vel_res) + "," + to_string(tau_dis) + "," +
                 to_string(tau_dis_hat);
    cout << row << endl;
    output_csv << row << endl;

    // update
    system.update(tau_ref - tau_dis);
  }
}
