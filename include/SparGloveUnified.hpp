#pragma once

#ifndef MEL_SPARGLOVE_HPP
#define MEL_SPARGLOVE_HPP

#include <MEL/Mechatronics/Robot.hpp>
#include <MEL/Mechatronics/Motor.hpp>
#include <MEL/Mechatronics/PdController.hpp>
#include <atomic>
#include <vector>

#include <MEL/Communications/MelShare.hpp>

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

#include <MEL/Math/Butterworth.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEL/Daq/encoder.hpp>
#include <MEL/Mechatronics/Amplifier.hpp>
#include <fstream>
#include <thread>
//#include <mutex>
#include <MEL/Utility/System.hpp>
#include <MEL/Utility/Mutex.hpp>
#include <MEL/Utility/NamedMutex.hpp>
#include <MEL/Utility/Lock.hpp>
#include <MEL/Devices/Myo/MyoBand.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEII/EMG/MesArray.hpp>

#include <MEL/Logging/Log.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEII/EMG/MesArray.hpp>
#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/System.hpp"
#include <MEII/EMG/MyoelectricSignal.hpp>
#include "MEL/Devices/Windows/Keyboard.hpp"
#include "MEII/Classification/EmgDirClassifier.hpp"
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Devices/Myo/MyoBand.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>
#include <MEII/EmgRealTimeControl/EmgRealTimeControl.hpp>
#include <MEL/Math/Butterworth.hpp>
//#include <SparGlove.hpp>


using namespace mel;
using namespace meii;

	//==============================================================================
	// Spar Glove driver adapted from HAPTIC PADDLE example
	//==============================================================================

	class SparGlove : public Robot {

	public:

		/// Constructor
		SparGlove();

		// Destructor
		~SparGlove();

		// Calibrates spar glove
		// void calibrate(volatile std::atomic<bool>& stop_flag);

		void start_myo();

		void start_turning();

		void notify(const std::size_t pred_label);

		//SparGlove(const SparGlove&);

		/// Overrides the default Robot::on_enable function with some custom logic
		bool on_enable() override;

		/// Overrides the default Robot::on_disable function with some custom logic
		bool on_disable() override;

	public:

		void set_pred_label(const std::size_t pred_label);

		size_t get_pred_label();

		Q8Usb q8_;
		std::vector<Encoder::Channel> encoders_;
		std::vector<Amplifier> amps_;
		std::vector<Motor> motors_;
		std::vector<PdController> pd_controllers_;

		// Parameters

		/// motor torque constants [Nm/A]
		std::array<double, 7> kt_;
		/// motor continous current limits [A]
		std::array<double, 7> motor_cont_limits_;
		/// motor peak current limits [A]
		std::array<double, 7> motor_peak_limits_;
		/// motor i^2*t times [s]
		std::array<Time, 7> motor_i2t_times_;
		/// transmission ratios [inch/inch]
		std::array<double, 7> eta_;
		/// encoder resolutions [counts/rev]
		std::array<uint32, 7> encoder_res_;
		/// joint position limits in negative rotation [rad]
		std::array<double, 7> pos_limits_neg_;
		/// joint position limits in positive rotation [rad]
		std::array<double, 7> pos_limits_pos_;
		/// joint velocity limits [rad/s]
		std::array<double, 7> vel_limits_;
		/// joint torque limits [Nm]
		std::array<double, 7> joint_torque_limits;
		/// joint kinetic friction [Nm]
		std::array<double, 7> kin_friction_;

		std::size_t pred_label;
		mel::Mutex mtx;

	private:

		


	};


#endif // MEL_SPARGLOVE_HPP