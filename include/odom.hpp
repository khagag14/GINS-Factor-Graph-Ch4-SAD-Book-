struct Odom 
{
    Odom(){}
    Odom( double timestamp, double left_pluse, double right_pluse) : 
                timestamp_(timestamp), left_pulse_(left_pluse), right_pulse_(right_pluse){}

    double timestamp_ = 0.0;
    double left_pulse_ = 0.0;
    double right_pulse_ = 0.0;
};