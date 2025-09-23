// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef __AC_PID_H__
#define __AC_PID_H__

#include <AP_Common.h>

#include <stdlib.h>
#include <math.h>               // for fabs()

#define AC_PID_ERROR_DTYPE float

/// @class	AC_PID
/// @brief	Object managing one PID control
class AC_PID {
public:

    /// Constructor for PID that saves its settings to EEPROM
    ///
    /// @note	PIDs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_imax    Initial value for the imax term.4
    ///
    AC_PID(
        const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const int16_t & initial_imax = 0.0)
    {
		//AP_Param::setup_object_defaults(this, var_info);

        _kp = initial_p;
        _ki = initial_i;
        _kd = initial_d;
        _imax = abs(initial_imax);

		// derivative is invalid on startup
		_last_derivative = NAN;
    }

    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    AC_PID_ERROR_DTYPE         get_pid(AC_PID_ERROR_DTYPE error, float dt);
    AC_PID_ERROR_DTYPE         get_pi(AC_PID_ERROR_DTYPE error, float dt);
    AC_PID_ERROR_DTYPE         get_p(AC_PID_ERROR_DTYPE error);
    AC_PID_ERROR_DTYPE         get_i(AC_PID_ERROR_DTYPE error, float dt);
    AC_PID_ERROR_DTYPE         get_d(AC_PID_ERROR_DTYPE error, float dt);
	AC_PID_ERROR_DTYPE 		get_leaky_i(AC_PID_ERROR_DTYPE error, float dt, float leak_rate);


    /// Reset the PID integrator
    ///
    void        reset_I();

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator        () (const float    p,
                             const float    i,
                             const float    d,
                             const int16_t  imaxval) {
        _kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
    }

    float        kP() const {
        return _kp;
    }
    float        kI() const {
        return _ki;
    }
    float        kD() const {
        return _kd;
    }
    int16_t        imax() const {
        return _imax;
    }

    void        kP(const float v)               {
        _kp = v;
    }
    void        kI(const float v)               {
        _ki = v;
    }
    void        kD(const float v)               {
        _kd = v;
    }
    void        imax(const int16_t v)   {
        _imax = abs(v);
    }

    float        get_integrator() const {
        return _integrator;
    }
    void        set_integrator(float i) {
        _integrator = i;
    }

    //static const struct AP_Param::GroupInfo        var_info[];

private:
    float       _kp;
    float       _ki;
    float       _kd;
    int16_t        _imax;

    float           _integrator;                                ///< integrator value
    AC_PID_ERROR_DTYPE         _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter

    /// Low pass filter cut frequency for derivative calculation.
    ///
    static const float  _filter;
};

#endif // __AC_PID_H__
