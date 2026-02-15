#include <iostream>                                                                                                       
#include <fstream>                                                                                                        
#include <cmath>
#include <Eigen/Dense>

// Constants
int main() {
// 
double n = 0.0;

// Missile Velocity, m/s
double VM = 3000.0;

// Target Velocity, m/s
double VT = 1000.0;

double XNT =-5;

// Heading Angle
double HEDEG = -40.0;

double XNP = 4;

// Initial Missile Position
double RMx = 0.0;
double RMy = 10000.0;

// Initial Target Position
double RTx = 40000.0;
double RTy = 10000.0;

// Beta Angle
double Beta = 0;

// Target Velocity
double VTx = -VT * std::cos(Beta);
double VTy = VT * std::sin(Beta);

// Heading Angle, rad
double HE = HEDEG * M_PI / 180;

double t = 0.0;
double S = 0.0;

// x distance between target and missile
double RTMx = RTx - RMx;

// y distance between target and missile
double RTMy = RTy - RMy;

// Total distance between Target & Missile
double RTM = std::sqrt(RTMx*RTMx + RTMy*RTMy);

// Lambda Angle
double XLAM = std::atan2(RTMy, RTMx);

// Leading Angle
double XLEAD = std::asin(VT * std::sin(Beta + XLAM) / VM);

// Theta Angle
double THET = XLAM + XLEAD;

// Initial Missile Velocity components
double VMx = VM * std::cos(THET + HE);
double VMy = VM * std::sin(THET + HE);

//
double VTMx = VTx - VMx;
double VTMy = VTy - VMy;

double Vc = -(RTMx * VTMx + RTMy * VTMy) / RTM;

std::vector<double> ArrayT;                                                                                        
std::vector<double> ArrayRTx;                                                                                      
std::vector<double> ArrayRTy;                                                                                      
std::vector<double> ArrayRMx;                                                                                      
std::vector<double> ArrayRMy;                                                                                      
std::vector<double> ArrayXNCG;                                                                                     
std::vector<double> ArrayRTM;     

double BetaD = 0.0;                                                                                                
double AMx = 0.0;                                                                                                  
double AMy = 0.0;                                                                                                  
double XNC = 0.0;          

while (Vc >= 0)
    {
        double H;
        double Beta_old;
        double RTx_old;
        double RTy_old;
        double RMx_old;
        double RMy_old;
        double VMx_old;
        double VMy_old;
        int step;
        int flag;
        if (RTM < 1000)
        {
            H = 0.0002;
        }else {
            H = 0.01;
        }
        Beta_old = Beta;
        RTx_old = RTx;
        RTy_old = RTy;
        RMx_old = RMx;
        RMy_old = RMy;
        VMx_old = VMx;
        VMy_old = VMy;
        step = 1;
        flag = 0;
        while (step <= 1){
            if (flag == 1){
                step = 2;
                Beta = Beta + H * BetaD;
                RTx = RTx + H * VTx;
                RTy = RTy + H * VTy;
                RMx = RMx + H * VMx;
                RMy = RMy + H * VMy;
                VMx = VMx + H * AMx;
                VMy = VMy + H * AMy;
                t = t + H; 
            }
            RTMx = RTx - RMx;
            RTMy = RTy - RMy;
            RTM = std::sqrt(RTMx * RTMx + RTMy * RTMy);
            VTMx = VTx - VMx;
            VTMy = VTy - VMy;
            Vc = -(RTMx * VTMx + RTMy * VTMy)/RTM;
            XLAM = std::atan2(RTMy, RTMx);
            double XLAMD = (RTMx * VTMy - RTMy * VTMx) / (RTM * RTM);
            XNC = XNP * Vc * XLAMD;
            AMx = -XNC * std::sin(XLAM);
            AMy = XNC * std::cos(XLAM);
            VTx = - VT * std::cos(Beta);
            VTy = VT * std::sin(Beta);
            BetaD = XNT / VT;
            flag = 1;
        }
    flag = 0;
    Beta = 0.5 * (Beta_old + Beta + H * BetaD);
    RTx = 0.5 * (RTx_old + RTx + H * VTx);
    RTy = 0.5 * (RTy_old + RTy + H * VTy);
    RMx = 0.5 * (RMx_old + RMx + H * VMx);
    RMy = 0.5 * (RMy_old + RMy + H * VMy);
    VMx = 0.5 * (VMx_old + VMx + H * AMx);
    VMy = 0.5 * (VMy_old + VMy + H * AMy);
    S = S + H;
    if (S >= 0.09999){
        S = 0.;
        n = n + 1;
        ArrayT.push_back(t);                                                                                           
        ArrayRTx.push_back(RTx);                                                                                       
        ArrayRTy.push_back(RTy);                                                                                       
        ArrayRMx.push_back(RMx);                                                                                       
        ArrayRMy.push_back(RMy);                                                                                       
        ArrayXNCG.push_back(XNC / 32.2);                                                                               
        ArrayRTM.push_back(RTM);        
        }
    }// End of while (Vc >= 0)
    
    // Print final miss distance
    std::cout << "Final RTM: " << RTM << std::endl;

    // Save Data to file
    std::ofstream outfile("datfil.txt");
    for (int i = 0; i < ArrayT.size(); i++) {
        outfile << ArrayT[i] << " "
                << ArrayRTx[i] << " "
                << ArrayRTy[i] << " "
                << ArrayRMx[i] << " "
                << ArrayRMy[i] << " "
                << ArrayXNCG[i] << " "
                << ArrayRTM[i] << "\n";
    }
    outfile.close();

    //Plot Missile and target positions.
    FILE*gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set title 'Two-dimensional tactical missile-target engagement simulation' \n");
    //fprintf(gp, "set yrange [0:11000]\n");
    fprintf(gp, "set xlabel 'Downrange (ft) '\n");
    fprintf(gp, "set ylabel 'Altitude (ft)'\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "plot 'datfil.txt' using 4:5 with lines title 'Missile', "
                "'datfil.txt' using 2:3 with lines title 'Target'\n");
    pclose(gp);

    std::cout << "*** Simulation Complete" << std::endl; 
}

