#include "energy.h"

#define AIR_DENSITY 1.225   //(kg/m^3)
#define GRAV 9.8            //(m/s^2)
#define PI 3.14159          //()
#define N_0 1e-9            //()
#define C 3e8               // Speed of light in m/s
#define N_0 1e-9            // power spectral dnesity white noise


//Calculate the hovering power (Watts)
double calcHoverPower(double mass, double radiusPropellers, double numbProp){
    return (sqrt(pow(((mass/1000)*GRAV), 3)/(2*PI*(radiusPropellers)*radiusPropellers*numbProp*AIR_DENSITY)));
}

//Calc vertical power
// mass = drone mass
// speed = vertical speed

double calcVertPower(double mass, double speed) {
    return ((mass/1000)*GRAV*speed);
}

//Calc drag coeff
double calcPDrag(double radiusPropellers, double numbProp, double dragCoeff, double vx, double vy){
    return ((dragCoeff*AIR_DENSITY*(numbProp*PI*radiusPropellers*radiusPropellers)*pow(((vx * vx) + (vy*vy)), 1.5))/8);
}

//Calculate the computing power ()
// y = switch capacitance constant (CPU)
// v = voltage
// cyclexop = cpu cycle per operation
// opxdata = numb operation per data
// Dn = numb of training sample
// I = local iter

double calcCompPower(double y, double v, double cyclexop, double opxdata, double Dn, double I){
    return ((y*pow(v, 2)*cyclexop)*opxdata*Dn*I)/v;
}

//Calculate data transmission rate
// B = Allocated bandwidth
// pn = Wireless transmission power
// Gn = Wireless channel gain (drone -> server)
// N0 = Power spectral density of white noise

double dataTransRate(double B, double pn, double Gn, double N0) {
    return B*(log2(1+((pn*Gn)*(N0*B))));
}

//Calculate communication power
// pn = Wireless transmission power
// sn = Size of local ML model
// rn = Data transmission rate
double calcCommPower(double pn, double sn, double rn){
    return (pn*sn)/rn;
}

//Calculate the drain power of hovering
double calcDrain(double power, double voltage){
    return power/voltage;
}

//Calc total power supply (Joules)
double calcJ(double volt, double charge){
    return volt * 3600 * charge;
}


//Test FPS
// 11.35 volts
// 60 frame per second
double test(double numCycle, double switchc, double freq){
    return ((numCycle*switchc*pow(freq, 2))/11.25)*60;
}


//COMMUNICATION

//DATA TRANSMIZZION RATE
// B = bandwith
// p_n = transmission power in watt
// f_c = 2.4e9 carrier distance freq
// d = 1000; distance
double calculate_rn(double B, double p_n, double f_c, double d) {

    // Calculate PL_LoS
    double PL_LoS = 20 * log10((4 * PI * f_c * d) / C);

    // Calculate G_n
    double G_n = pow(10, -(PL_LoS / 10));

    // Calculate r_n
    double r_n = B * log2(1 + (p_n * G_n) / (N_0 * B));

    return r_n;
}


// Function to calculate (p_n * s_n) / r_n
//p_n = wireless transmission power = 0.1;      // Transmission power in watts (100 mW)
//s_n = size of local ML model      = 1e6 (1MB)
double calcCommEnergy(double p_n, double s_n, double B, double f_c, double d) {
    return ((p_n * s_n) / calculate_rn(B, p_n, f_c, d))/12.6;
}
