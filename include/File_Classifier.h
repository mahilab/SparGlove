#pragma once
#ifndef File_CLASSIFIER_HPP
#define File_CLASSIFIER_HPP

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
#include <SparGlove.hpp>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdio.h>



class FileClassifier {

public:
	//Constructor
	FileClassifier(std::vector<std::string> sample_files); // , SparGlove& sg);

															//Destructor
	~FileClassifier();

	int*** Parse(std::string);



};

#endif // !MYO_CLASSIFIER_HPP
