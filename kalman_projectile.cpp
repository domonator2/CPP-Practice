#include <iostream>
#include <cmath>
#include <random>
#include <vector>

// Constants

// [m/s^2] Standard Gravity
const double g0 = 9.80665;

// [s] Time Step
const double dt = 0.01;

// [m/s] Initial Velocity
const double v0 = 50.0;

// [degrees] Launch Angle
const double theta_deg = 45.0;

// [-] Measurement Noise
const double MEAS_NOISE = 5.0;

// [-] Process Noise
const double PROC_NOISE = 0.1;

// Initialize State Vector
double S0[4][1] = {
    {0}, 
    {0}, 
    {v0 * std::cos(M_PI/180 * theta_deg)}, 
    {v0 * std::sin(M_PI/180 * theta_deg)}};

// Define Matrix
struct KalmanFilter {
      std::vector<std::vector<double>> State;
      std::vector<std::vector<double>> P;
      std::vector<std::vector<double>> F;
      std::vector<std::vector<double>> H;
      std::vector<std::vector<double>> K;
      std::vector<std::vector<double>> R;
      std::vector<std::vector<double>> Q;

      // Constructor - runs when you create a KalmanFilter
      KalmanFilter() {
          State = std::vector<std::vector<double>>(4, std::vector<double>(1, 0));
          P = std::vector<std::vector<double>>(4, std::vector<double>(4, 0));
          F = std::vector<std::vector<double>>(4, std::vector<double>(4, 0));
          H = std::vector<std::vector<double>>(2, std::vector<double>(4, 0));
          K = std::vector<std::vector<double>>(4, std::vector<double>(2, 0));
          R = std::vector<std::vector<double>>(2, std::vector<double>(2, 0));
          Q = std::vector<std::vector<double>>(4, std::vector<double>(4, 0));
      }
  };

std::vector<std::vector<double>> multiply(
      const std::vector<std::vector<double>>& Mat1,
      const std::vector<std::vector<double>>& Mat2){

        int rows1 = Mat1.size();
        int cols1 = Mat1[0].size();
        int cols2 = Mat2[0].size();

        // Create the multiplied Matrix

        std::vector<std::vector<double>> MultMat(rows1, std::vector<double>(cols2,0));

        for (int i = 0; i < rows1; i++){
            for (int j = 0; j < cols2; j++){
                for (int k = 0; k < cols1; k++){
                    MultMat[i][j] += Mat1[i][k] * Mat2[k][j];
                }
            }
        }
        return MultMat;
      }

std::vector<std::vector<double>> add(
      const std::vector<std::vector<double>>& Mat1,
      const std::vector<std::vector<double>>& Mat2){

        int rows1 = Mat1.size();
        int rows2 = Mat2.size();
        int cols1 = Mat1[0].size();
        int cols2 = Mat2[0].size();

        if (rows1 != rows2 || cols1 != cols2){
             throw std::invalid_argument("Matrix dimensions must match for addition");
        }

        // 
        std::vector<std::vector<double>> addMat(rows1, std::vector<double>(cols1,0));

        // Start addition loop
        for (int i = 0; i < rows1; i++){
            for (int j = 0; j < cols1; j++){
                addMat[i][j] = Mat1[i][j] + Mat2[i][j];
            } 
        }
        
        return addMat;
      }

std::vector<std::vector<double>> subtract(
      const std::vector<std::vector<double>>& Mat1,
      const std::vector<std::vector<double>>& Mat2){

        int rows1 = Mat1.size();
        int rows2 = Mat2.size();
        int cols1 = Mat1[0].size();
        int cols2 = Mat2[0].size();

        if (rows1 != rows2 || cols1 != cols2){
             throw std::invalid_argument("Matrix dimensions must match for subtraction");
        }

        // Initialize subtraction vector
        std::vector<std::vector<double>> subMat(rows1, std::vector<double>(cols1,0));

        // Start subtraction loop
        for (int i = 0; i < rows1; i++){
            for (int j = 0; j < cols1; j++){
                subMat[i][j] = Mat1[i][j] - Mat2[i][j];
            } 
        }
        
        return subMat;
      }

// Transpose function
std::vector<std::vector<double>> transpose(
      const std::vector<std::vector<double>>& Mat1){

        int rows1 = Mat1.size();
        int cols1 = Mat1[0].size();

        // Initialize transpose vector
        std::vector<std::vector<double>> transMat(cols1, std::vector<double>(rows1,0));

         // Start Tranpose loop
        for (int i = 0; i < rows1; i++){
            for (int j = 0; j < cols1; j++){
                
                transMat[j][i] = Mat1[i][j];

            }
        }
        return transMat;
    }

// Inverse Function
std::vector<std::vector<double>> inverse(
      const std::vector<std::vector<double>>& Mat1){

        int rows1 = Mat1.size();
        int cols1 = Mat1[0].size();
        
        // Initialize Inverse Matrix
        std::vector<std::vector<double>> invMat(rows1, std::vector<double>(cols1,0));

        if (rows1 != cols1){
             throw std::invalid_argument("Matrix dimensions must match for inverse");
        }

        double det = Mat1[0][0] * Mat1[1][1] - Mat1[0][1] * Mat1[1][0];

        // Check if the determinant is zero
        if(det == 0){
            throw std::invalid_argument("Determinant is zero");
        }

        if (rows1 != 2 || cols1 != 2){
            throw std::invalid_argument("Only 2x2 inverse is supported");
        }
        
        double a = Mat1[0][0];
        double b = Mat1[0][1];
        double c = Mat1[1][0];
        double d = Mat1[1][1];
        
        invMat[0][0] = d / det;
        invMat[0][1] = -b / det;
        invMat[1][0] = -c / det;
        invMat[1][1] = a / det;

        return invMat;

        }

void predict(KalmanFilter& kf, std::vector<std::vector<double>>& G) {
    kf.State = add(multiply(kf.F, kf.State), G);
    kf.P = add(multiply(multiply(kf.F, kf.P), transpose(kf.F)), kf.Q);    
}

void update(KalmanFilter& kf, const std::vector<std::vector<double>>& z){
    std::vector<std::vector<double>> y = subtract(z, multiply(kf.H, kf.State));
    std::vector<std::vector<double>> S = add(multiply(multiply(kf.H, kf.P), transpose(kf.H)), kf.R);
    kf.K = multiply(multiply(kf.P, transpose(kf.H)), inverse(S));
    kf.State = add(kf.State, multiply(kf.K, y));
    int dim = kf.State.size();

    // Initialize identity Matrix
    std::vector<std::vector<double>> I(dim, std::vector<double>(dim,0));

    // Define the Identity Matrix.
    for (int i = 0; i < dim; i++){
        I[i][i] = 1;
    }
    kf.P = multiply(subtract(I, multiply(kf.K, kf.H)), kf.P);
}

int main() { 

//Create Instance of Kalman Filter Struct
KalmanFilter kf;
    
// Access the F Matrix
kf.F = {
    {1, 0, dt, 0},
    {0, 1, 0, dt},
    {0, 0, 1, 0},
    {0, 0, 0, 1}};

kf.H = {
      {1, 0, 0, 0},
      {0, 1, 0, 0}
  };

kf.R = {
      {MEAS_NOISE * MEAS_NOISE, 0},
      {0, MEAS_NOISE * MEAS_NOISE}
  };

kf.Q = {
      {PROC_NOISE, 0, 0, 0},
      {0, PROC_NOISE, 0, 0},
      {0, 0, PROC_NOISE, 0},
      {0, 0, 0, PROC_NOISE}
  };

kf.P = {
      {100, 0, 0, 0},
      {0, 100, 0, 0},
      {0, 0, 100, 0},
      {0, 0, 0, 100}
  };

kf.State = {
      {0},
      {0},
      {v0 * std::cos(M_PI/180 * theta_deg)},
      {v0 * std::sin(M_PI/180 * theta_deg)}
  };

std::vector<std::vector<double>> G = {
      {0},
      {-0.5 * g0 * dt * dt},
      {0},
      {-g0 * dt}
  };

    // Random Number Generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0, MEAS_NOISE);

    // Set initial Conditions
    double true_x = 0;
    double true_y = 0;
    double true_vx = v0 * std::cos(M_PI / 180 * theta_deg);
    double true_vy = v0 * std::sin(M_PI / 180 * theta_deg);

    // Time
    double t = 0;

    while(true_y >= 0){

        // True physics
        true_x += true_vx * dt;
        true_y += true_vy * dt - 0.5 * g0 * dt * dt;
        true_vy -= g0 * dt;

        // Create Noise
        std::vector<std::vector<double>> z = {
            {true_x + noise(gen)},
            {true_y + noise(gen)}
        };

        // Predict + Update
        predict(kf, G);
        update(kf, z);

        // 4. Print comparison
      std::cout << "True: (" << true_x << ", " << true_y << ")  "
                << "Est: (" << kf.State[0][0] << ", " << kf.State[1][0] << ")\n";

        t += dt;
    }

}
