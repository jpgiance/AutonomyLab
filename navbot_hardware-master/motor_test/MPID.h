#ifndef MPID_h
#define MPID_h


class MPID {
  public:
    // Constructor
    MPID(double*, double*, double*, double, double, double); 

    bool calculate(); 
    void setConstants(double, double, double);
    void setSampleTime(int);
    void setOutputLimits(double, double);


  private:
    void init();

    double _kp, _ki, _kd; // PID coefficients
    double _integral, _previousError; // To store integral and last error for calculations.
    unsigned long _lastTime; // To store the last time the PID was calculated.

    double *_input, *_output, *_setpoint;

	  double _outputSum, _lastInput;

	  unsigned long _sampleTime;
	  double _outMin, _outMax;

};

#endif