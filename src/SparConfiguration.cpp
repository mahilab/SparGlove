#include "OwConfiguration.hpp"

namespace mel {

	//==============================================================================
	// CLASS DEFINTIONS
	//==============================================================================

	SparConfiguration::SparConfiguration(
		DaqBase& daq,
		Watchdog& watchdog,
		const std::vector<Logic>& enable_levels,
		const std::vector<DigitalOutput::Channel>& enable_channels,
		const std::vector<double>& command_gains,
		const std::vector<AnalogOutput::Channel>& command_channels,
		const std::vector<Limiter>& amp_current_limiters,
		const std::vector<Logic>& fault_levels,
		const std::vector<DigitalInput::Channel>& fault_channels,
		const std::vector<double>& sense_gains,
		const std::vector<AnalogInput::Channel>& sense_channels,
		const std::vector<Encoder::Channel>& encoder_channels)
		: daq_(daq),
		watchdog_(watchdog),
		encoder_channels_(encoder_channels) {
		for (std::size_t i = 0; i < 7; ++i) {
			amplifiers_.push_back(
				Amplifier("spar_amp_" + std::to_string(i), enable_levels[i],
					enable_channels[i], command_gains[i], command_channels[i],
					amp_current_limiters[i], fault_levels[i],
					fault_channels[i], sense_gains[i], sense_channels[i]));
		}
	}

	SparConfiguration::SparConfiguration(
		DaqBase& daq,
		Watchdog& watchdog,
		const std::vector<Encoder::Channel>& encoder_channels,
		const std::vector<Amplifier>& amplifiers)
		: daq_(daq),
		watchdog_(watchdog),
		encoder_channels_(encoder_channels),
		amplifiers_(amplifiers) 
	{
	
	}

}  // namespace mel

