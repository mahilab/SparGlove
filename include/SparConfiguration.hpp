#pragma once

#ifndef MEL_SPARCONFIGURATION_HPP
#define MEL_SPARCONFIGURATION_HPP

#include <MEL/Mechatronics/Amplifier.hpp>
#include <MEL/Daq/DaqBase.hpp>
#include <MEL/Daq/Encoder.hpp>
#include <MEL/Daq/Input.hpp>
#include <MEL/Daq/Output.hpp>
#include <MEL/Daq/Watchdog.hpp>
#include <vector>


namespace mel {

	//==============================================================================
	// FORWARD DECLARATIONS
	//==============================================================================

	class SparGlove;
	class Q8Usb;

	//==============================================================================
	// CLASS DECLARATION
	//==============================================================================

	/// Encapsulates the hardware configuration for an OpenWrist
	struct SparConfiguration {
	public:
		/// Generic Configuration 1 (creates Amplifiers)
		SparConfiguration(DaqBase& daq,
			Watchdog& watchdog,
			const std::vector<Logic>& enable_levels,
			const std::vector<DigitalOutput::Channel>& enable_channels,
			const std::vector<double>& command_gains,
			const std::vector<AnalogOutput::Channel>& command_channels,
			const std::vector<Limiter>& amp_current_limiters,
			const std::vector<Logic>& fault_levels,
			const std::vector<DigitalInput::Channel>& fault_channels,
			const std::vector<double>& sense_gains,
			const std::vector<AnalogInput::Channel>& sense_channel,
			const std::vector<Encoder::Channel>& encoder_channels);

		/// Generic Configuration 2 (given Amplifiers)
		SparConfiguration(DaqBase& daq,
			Watchdog& watchdog,
			const std::vector<Encoder::Channel>& encoder_channels,
			const std::vector<Amplifier>& amplifiers);


	private:

		friend class OpenWrist;

		DaqBase& daq_;                                     ///< DAQ controlling the OpenWrist
		Watchdog& watchdog_;                               ///< watchdog the OpenWrist is guarded by
		std::vector<Encoder::Channel>  encoder_channels_;  ///< encoder channels that measure motor positions
		std::vector<Amplifier> amplifiers_;                ///< the amplifiers being use to control the OpenWrist's motors
	};

}  // namespace mel

#endif  // MEL_SPARCONFIGURATION_HPP

   //==============================================================================
   // CLASS DOCUMENTATION
   //==============================================================================