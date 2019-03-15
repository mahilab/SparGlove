
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
#include <MEII/Utility/DataLogger.hpp>

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

using namespace mel;
using namespace meii;

int main() {
	std::vector <std::string> training_files;
	std::string training_dir = "../../build/";
	//training_files = { "PoseButtonTraining.csv", "PoseExtensionTraining.csv", "PoseOKTraining.csv", "PosePointTraining.csv", "PoseCylinderTraining.csv" ,  "PoseThumbOppoTraining.csv", "PoseThumbsUpTraining.csv" };

	training_files = { "SCI_Point_Training.csv", 
					   "SCI_Extension_Training.csv", 
					   "SCI_Oppo_Training.csv", 
					   "SCI_ok_Training.csv", 
					   "SCI_ThumbsUp_Training.csv" ,  
					   "SCI_Cylinder_Training.csv", 
					   "SCI_Button_Training.csv" };	

	// handle inputs
	std::vector<uint32> emg_channel_numbers = { 0,1,2,3,4,5,6,7 };
	std::cout << "Made it this far" << std::endl;
	mel::MyoBand myo("my_myo");



	// construct array of Myoelectric Signals
	MesArray mes(myo.get_channels(emg_channel_numbers));

	// make MelShares
	MelShare ms_mes_env("mes_env");
	MelShare ms_mes_dm("mes_dm");
	MelShare ms_pred_label("pred_label");

	// initialize testing conditions
	Time Ts = milliseconds(1); // sample period
	std::size_t num_classes = 7; // number of active classes
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };
	Time mes_active_capture_period = seconds(0.2);
	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
	mes.resize_buffer(mes_active_capture_window_size);
	std::size_t pred_label = 0;
	std::size_t prev_pred_label = 0;

	
	// initialize classifier
	bool RMS = true;
	bool MAV = false;
	bool WL = false;
	bool ZC = false;
	bool SSC = false;
	bool AR1 = true;
	bool AR2 = true;
	bool AR3 = true;
	bool AR4 = true;
	EmgDirClassifier dir_classifier(num_classes, emg_channel_numbers.size(), Ts, RMS, MAV, WL, ZC, SSC, AR1, AR2, AR3, AR4, seconds(0.4), seconds(0.2), seconds(0.3));

	bool run = false;
	Clock cooldown_clock;

	// construct clock to regulate interaction
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds());

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);


	//int*** read_in_object = new int**[7];
	//std::vector<std::vector<std::vector<double>>> read_in_object;
	
	std::vector<std::vector<double>> training_file_data;
	
	//std::vector<std::vector<double>> read_in_object;
	
	for (int pose = 0; pose < training_files.size(); pose++) {
		//read_in_object = std::vector<std::vector<std::vector<double>>>(7);
		DataLogger::read_from_csv(training_file_data, training_dir + training_files[pose]);
	
			if (dir_classifier.add_training_data(pose, training_file_data)) {
				LOG(Info) << "Added active data for target " + stringify(pose + 1) + ".";
			}
	
	}

	// prompt the user for input
	print("Press 'A + target #' to add training data for one target.");
	print("Press 'C + target #' to clear training data for one target.");
	print("Press 'T' to train classifier and begin real-time classification.");
	print("Press 'U' to use the classifier on a pre-existing data file");
	print("Press 'R' to report the boolean settings");
	print("Press 'Escape' to exit.");

	myo.enable();

	std::vector<std::vector<double>> w;
	std::vector<double> w_0;
	std::vector<double> y(7);

	while (true) {
	
		// clear active data
		for (std::size_t k = 0; k < num_classes; ++k) {
			if (Keyboard::are_all_keys_pressed({ Key::C, active_keys[k] })) {
				if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
					if (dir_classifier.clear_training_data(k)) {
						LOG(Info) << "Cleared active data for target " + stringify(k + 1) + ".";
					}
					keypress_refract_clock.restart();
				}
			}
		}

		// capture active data
		for (std::size_t k = 0; k < num_classes; ++k) {
			if (Keyboard::are_all_keys_pressed({ Key::A, active_keys[k] })) {
				if (mes.is_buffer_full()) {
					if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
						if (dir_classifier.add_training_data(k, mes.get_dm_buffer_data(mes_active_capture_window_size))) {
							LOG(Info) << "Added active data for target " + stringify(k + 1) + ".";
						}
						keypress_refract_clock.restart();
					}
				}
			}
		}

		// train the active/rest classifiers
		if (Keyboard::is_key_pressed(Key::T)) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				if (dir_classifier.train()) {
					LOG(Info) << "Trained new active/rest classifier based on given data.";
					dir_classifier.save();
				}
				keypress_refract_clock.restart();
			}
		}


		// train the active/rest classifiers
		if (Keyboard::is_key_pressed(Key::R)) {
			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
				std::cout << "Boolean settings are: ";
				std::cout << "RMS = " << RMS << ", MAV = " << MAV << ", WL = " << WL << ", ZC = " << ZC << ", SSC = " << SSC << ", AR1 = " << AR1 << ", AR2 = " << AR2 << ", AR3 = " << AR3 << ", AR4 = " << AR4;
				}
				keypress_refract_clock.restart();
			}

		if (Keyboard::is_key_pressed(Key::U)) {
			if (dir_classifier.is_trained()) {
				
				std::string file_to_parse;
				std::cout << "Which file would you like to classify?" << std::endl;
				std::getline(std::cin, file_to_parse);
				//pred_label = 9;
				//std::cout << pred_label << std::endl;

				std::vector<std::vector<double>> test_file_data;
				DataLogger::read_from_csv(test_file_data, training_dir +file_to_parse);
				for (int i = 0; i < test_file_data.size(); i++) {
					dir_classifier.update(test_file_data[i]);
				}
						
				pred_label = dir_classifier.get_class();

				//dir_classifier.get_model(w, w_0);
				y = dir_classifier.get_model_output();


				//if (dir_classifier.update(butt_class)) {
				std::cout << "And the winner is..." ;
				std::cout << pred_label << std::endl;
				
				//int bucket = dir_classifier.get_class();
				
				switch (pred_label)
				{
				case 0: std::cout << "Pointing" << std::endl;
					break;
				case 1: std::cout << "Extension (outstretched palm)" << std::endl;
					break;
				case 2: std::cout << "Thumb Opposition" << std::endl;
					break;
				case 3: std::cout << "OK Sign (i.e. Pinch)" << std::endl;
					break;
				case 4: std::cout << "Thumbs Up (Hook)" << std::endl;
					break;
				case 5: std::cout << "Cylinder Grasp" << std::endl;
					break;
				case 6: std::cout << "Button Press" << std::endl;
					break;
				}
				//std::cout << pred_label << std::endl;

				 
				std::cout << "Confidence Measurements:" << std::endl;
				std::cout << "Get Model Output:" << std::endl;
				std::cout << y << std::endl;
				//std::cout << w_0 << std::endl;
				y = dir_classifier.get_class_posteriors();
				std::cout << "Get Class Posteriors:" << std::endl;
				std::cout << y << std::endl;
				std::cout << " " << std::endl;

			}
			else
				std::cout << "You must train the algorithm before attempting to Use a data set" << std::endl;
		}

		// write to MelShares
		ms_mes_env.write_data(mes.get_envelope());
		ms_mes_dm.write_data(mes.get_demean());
		ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			//ctrlc = true;
		}

		// wait for remainder of sample period
		timer.wait();

	} // end control loop

	return 0;
}

