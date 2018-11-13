#pragma once

#include <MEL/Communications/MelShare.hpp>
#include <MEL/Mechatronics/Motor.hpp>
#include <MEL/Mechatronics/Robot.hpp>
#include <MEL/Mechatronics/Amplifier.hpp>
#include <MEL/Mechatronics/PositionSensor.hpp>

#include <MEL/Mechatronics/VirtualVelocitySensor.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Daq/InputOutput.hpp>
#include <MEL/Daq/Input.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Constants.hpp>
#include <MEL/Math/Differentiator.hpp>
#include <MEL/Math/Filter.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Mechatronics/PdController.hpp>
#include <MEL/Math/Butterworth.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEL/Daq/encoder.hpp>
#include <fstream>
#include <thread>

using namespace mel;

//==============================================================================
// Spar Glove driver adapted from HAPTIC PADDLE example
//==============================================================================

class SparGlove : public Robot {

public:

	/// Constructor
	SparGlove(DigitalOutput::Channel d_o,
		AnalogOutput::Channel ao,
		AnalogInput::Channel ai,
		Encoder::Channel enc) :
		// Robot constructor
		Robot("spar_glove"),
		// init amplifier
		amp_("amc_12a8", High, d_o, -1.065, ao),
		// init motor
		motor_("maxon_", 0.0229, amp_, Limiter(1.8821, 12.0, seconds(1))),
		//init position sensor
		encoder_(enc)
	{
		//create joint
		add_joint(Joint("spar_joint_0", &motor_, 0.713 / 6.250, &encoder_, 1.0,
			&encoder_, 1.0, { -50 * DEG2RAD, 50 * DEG2RAD },
			500 * DEG2RAD, 1.0));
	}



private:

	/// Overrides the default Robot::enable function with some custom logic
	bool on_enable() override {
		return true;
	}

private:
	Amplifier amp_;
	Motor motor_;
	Encoder::Channel encoder_;
};