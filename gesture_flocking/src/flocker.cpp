#include<flocker.hpp>


class FlockerClass
{
    public:
        FlockerClass(ros::NodeHandle* nodehandle);

    private:
        proximalPar ProxParameters;
        motionControlPar motionParameters;


        double getProximalMagnitude(double range);

        void initializeSubscribers();
        void neighborCallback();


};


double FlockerClass::getProximalMagnitude(double range)
{
    double proximalMag =    -4*ProxParameters._steepness_potential_ * ProxParameters._strength_potential_ / range*
                            (2*pow(ProxParameters.noise_ / range, 2*ProxParameters._steepness_potential_) - 
                            pow(ProxParameters.noise_ / range, ProxParameters._steepness_potential_));
    return proximalMag;

}


