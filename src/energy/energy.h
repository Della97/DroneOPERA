#ifndef ENERGY_H
#define ENERGY_H

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>


//Calculate the hovering power (Watts)
//Mass             (Kg)
//RadiusPropellers (m)
//numbProp         ()
//==>
//Watts


double vectorMagnitude2D(double vx, double vy);

double calculateOmega(double vx, double vy);

double P_level(double mass, double radiusPropellers, double numbProp, double vx, double vy);

double P_vertical(double mass, double vz);

double P_drag(double dragCoeff, double radiusPropellers, double numbProp, double vx, double vy);

double P_UAV(double mass, double dragCoeff, double radiusPropellers, double numbProp, double vx, double vy, double vz);

//*******************************************************************************************************************

//Calculate data transmission rate (rn)
// B = Allocated bandwidth
// pn = Wireless transmission power
// Gn = Wireless channel gain (drone -> server)
// N0 = Power spectral density of white noise

double sataTransRate(double B, double pn, double Gn, double N0);

//Calculate communication power
// pn = Wireless transmission power
// sn = Size of local ML model
// rn = Data transmission rate

double calcCommPower(double pn, double sn, double rn);

//Calculate the drain of the battery while overing (Ampere)
//power   (Watt)
//voltage (V)
//==>
//Ampere
double calcDrain(double power, double voltage);

//Calculate total power in Joules

double calcJ(double volt, double charge);

//Test FPS
double test(double numCycle, double switchc, double freq);



//communication

//DATA TRANSMIZZION RATE
// B = bandwith
// p_n = 0.1;      // Transmission power in watts (100 mW)
// f_c = 2.4e9 carrier distance freq
// d = 1000; distance
// xi_LoS = 3 excessive path loss in dB
// N_0 = 1e-9 power spectral density W/Hz
double calculate_rn(double B, double p_n, double f_c, double d, double xi_LoS, double N_0);


// Function to calculate (p_n * s_n) / r_n
//p_n = wireless transmission power = 0.1;      // Transmission power in watts (100 mW)
//s_n = size of local ML model      = 1e6 (1MB)
double calcCommEnergy(double p_n, double s_n, double B, double f_c, double d);


//Comp power
double calcCompPower(double y, double v, double cyclexop, double opxdata, double Dn, double I);


#endif // ENERGY_H