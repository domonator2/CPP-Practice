#include <iostream>                                                                                                       
#include <fstream>                                                                                                        
#include <cmath>                                                                                                          
                                                                                                                            
int main() {                                                                                                              
    // 1. Run your simulation and write data                                                                              
    std::ofstream file("trajectory.csv");                                                                                 
    for (double t = 0; t < 10.0; t += 0.01) {                                                                             
        double x = 300.0 * t;                                                                                             
        double y = 300.0 * t - 0.5 * 9.81 * t * t;                                                                        
        if (y < 0) break;                                                                                                 
        file << t << " " << x << " " << y << "\n";                                                                        
    }                                                                                                                     
    file.close();                                                                                                         
                                                                                                                            
    // 2. Plot it                                                                                                         
    FILE* gp = popen("gnuplot -persist", "w");                                                                            
    fprintf(gp, "set title 'Projectile Trajectory'\n");                                                                   
    fprintf(gp, "set xlabel 'Range (m)'\n");                                                                              
    fprintf(gp, "set ylabel 'Altitude (m)'\n");                                                                           
    fprintf(gp, "set grid\n");                                                                                            
    fprintf(gp, "plot 'trajectory.csv' using 2:3 with lines title 'Trajectory'\n");                                       
    pclose(gp);                                                                                                           
                                                                                                                            
    return 0;                                                                                                             
}                                          