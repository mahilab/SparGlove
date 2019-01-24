#include "MYO_Classifier.hpp"
using namespace mel;
using namespace meii;

//ctrl_bool ctrlc(false);
//bool handler(CtrlEvent event) {
//	ctrlc = true;
//	return true;
//}

//ctrl_bool cancel(false);
//bool handler(CtrlEvent event) {
//    cancel = true;
//    return true;
//}

//class MYOClassifier {
class CSVRow
{
public:
	std::string const& operator[](std::size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}
	void readNextRow(std::istream& str)
	{
		std::string         line;
		std::getline(str, line);

		std::stringstream   lineStream(line);
		std::string         cell;
		m_data.clear();
		while (std::getline(lineStream, cell, ','))
		{
			m_data.push_back(cell);
		}
		// This checks for a trailing comma with no data after it.
		if (!lineStream && cell.empty())
		{
			// If there was a trailing comma then add an empty element.
			m_data.push_back("");
		}
	}

private:
	std::vector<std::string>    m_data;

};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
	data.readNextRow(str);
	return str;
}

class CSVIterator
{
public:
	typedef std::input_iterator_tag     iterator_category;
	typedef CSVRow                      value_type;
	typedef std::size_t                 difference_type;
	typedef CSVRow*                     pointer;
	typedef CSVRow&                     reference;

	CSVIterator(std::istream& str) :m_str(str.good() ? &str : NULL) { ++(*this); }
	CSVIterator() :m_str(NULL) {}

	// Pre Increment
	CSVIterator& operator++() { if (m_str) { if (!((*m_str) >> m_row)) { m_str = NULL; } }return *this; }
	// Post increment
	CSVIterator operator++(int) { CSVIterator    tmp(*this); ++(*this); return tmp; }
	CSVRow const& operator*()   const { return m_row; }
	CSVRow const* operator->()  const { return &m_row; }

	bool operator==(CSVIterator const& rhs) { return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL))); }
	bool operator!=(CSVIterator const& rhs) { return !((*this) == rhs); }
private:
	std::istream*       m_str;
	CSVRow              m_row;
};

MYOClassifier::MYOClassifier(std::vector<std::string> training_files, SparGlove& sg) {
	
	

	// handle inputs
	std::vector<uint32> emg_channel_numbers = { 0,1,2,3,4,5,6,7 };

	mel::MyoBand myo("my_myo");

	// initialize logger
	//mel::init_logger(mel::Verbose);

	// register ctrl-c handler
	//register_ctrl_handler(handler);

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
	bool AR1 = false;
	bool AR2 = false;
	bool AR3 = false;
	bool AR4 = false;
	EmgDirClassifier dir_classifier(num_classes, emg_channel_numbers.size(), Ts, RMS, MAV, WL, ZC, SSC, AR1, AR2, AR3, AR4, seconds(1.0), seconds(0.2), seconds(0.9));

	bool run = false;
	Clock cooldown_clock;

	// construct clock to regulate interaction
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds());

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);

	//{
	//	Lock lock(sg.mtx);
	//	std::cout << "MYO_Classifier thinks pred_label is" << std::endl;
//
//		std::cout << pred_label << std::endl;
//	}
	


	//std::cout << mes_active_capture_window_size << std::endl;

	int pose = 0;

	for (std::vector<std::string>::iterator it = training_files.begin(); it != training_files.end(); ++it) {
		int*** training_data = Parse(*it);
		
		for (int k = 0; k <= 6; k++) {
			std::vector<std::vector<double>> training_set;

			for (int L = 0; L < 200; L++) {
				std::vector<double> row;

				for (int m = 0; m < 8; m++) {
					
					row.push_back(training_data[k][L][m]);  //converts integers in input file to double precision which RealTimeClassifier expects
				}
				training_set.push_back(row);
			}
			dir_classifier.add_training_data(pose, training_set);

			std::cout << training_set.size() << std::endl;

			if (dir_classifier.add_training_data(pose, training_set)) {
				LOG(Info) << "Added active data for target " + stringify(pose + 1) + ".";
			}
			
		}
		pose++;
		
	}

	// prompt the user for input
	print("Press 'A + target #' to add training data for one target.");
	print("Press 'C + target #' to clear training data for one target.");
	print("Number of targets/classes is:");
	print(num_classes);
	print("Press 'T' to train classifier and begin real-time classification.");
	print("Press 'U' to use the classifier on a pre-existing data file");
	print("Press 'R' to run the OpenWrist");
	print("Press 'S' to stop the OpenWrist");
	print("Press 'Escape' to exit.");

	// enable hardware
	//q8.enable();
	//ow.enable();
	//q8.watchdog.start();
	myo.enable();
	sg.enable();

	while (true) {
		//counter = counter + 1;

		//std::cout << counter << std::endl;
		// update hardware
		//q8.watchdog.kick();
		//q8.update_input();
		
		{
			Lock lock(sg.mtx);
//			print("MYOClassifier Main loop");
		}
//		std::cout << "Got to the main loop" << std::endl;
		
		// update all DAQ input channels
		myo.update();

		// emg signal processing
		mes.update_and_buffer();
		
		// predict state
		if (dir_classifier.update(mes.get_demean())) {
			pred_label = dir_classifier.get_class();
			sg.notify(pred_label);
			//std::cout << "MYO_Classifier thinks pred_label is" << std::endl;

			//std::cout << dir_classifier.get_class() << std::endl;

	//		std::cout << "Counter is " << std::endl;

		//	std::cout << counter	<< std::endl;
		}

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
				}
				keypress_refract_clock.restart();
			}
			
		}

		if (Keyboard::is_key_pressed(Key::U)) {
			if (dir_classifier.is_trained()) {
				
				std::string file_to_parse;
				std::cout << "Which file would you like to classify?" << std::endl;
				std::getline(std::cin, file_to_parse);
			
				//for (std::vector<std::string>::iterator it = file_to_parse.begin(); it != file_to_parse.end(); ++it) {
					int*** training_data = Parse(file_to_parse);

						std::vector<std::vector<double>> classification_set;

						for (int L = 0; L < 200; L++) {
							std::vector<double> row;

							for (int m = 0; m < 8; m++) {

								row.push_back(training_data[0][L][m]);  //converts integers in input file to double precision which RealTimeClassifier expects
							}
							classification_set.push_back(row);
						}
						//dir_classifier.add_training_data(pose, classification_set);
						/*
						std::cout << classification_set.size() << std::endl;

						Butterworth filter(8, hertz(1000), .05);
					    
						std::vector<double> butt_class;

						for (int i = 0; i < 200; i++) {
							butt_class[i] = filter.update(classification_set[1][i]);
						}

						if (dir_classifier.update(butt_class)) {
							std::cout << "And the winner is..." << std::endl;
							int bucket = dir_classifier.get_class();
							std::cout << bucket << std::endl;

							std::cout << "We've made a prediction!" << std::endl; 
						}*/

			}
			else
				std::cout << "You must train the algorithm before attempting to Use a data set" << std::endl;
		}

		// set OpenWrist run state
		if (Keyboard::is_key_pressed(Key::R) && keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
			LOG(Info) << "OpenWrist Running";
			run = true;
			keypress_refract_clock.restart();
		}
		else if (Keyboard::is_key_pressed(Key::S) && keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
			LOG(Info) << "OpenWrist Stopped";
			run = false;
			keypress_refract_clock.restart();
		}

	
	   // set OpenWrist goal
		if (run) {
			//    if (cooldown_clock.get_elapsed_time() > seconds(0.5)) {
			//        if (pred_label == 0) {
			//            // Rest
			//            ps_goal = ow[0].get_position();
			//            fe_goal = ow[1].get_position();
			//            ru_goal = ow[2].get_position();
			//        }
			//        if (pred_label == 1) {
			//            // Flexion
			//            ps_goal = ow[0].get_position();
			//            fe_goal = 60 * mel::DEG2RAD;
			//            ru_goal = ow[2].get_position();
			//        }
			//        else if (pred_label == 2) {
			//            // Extension
			//            ps_goal = ow[0].get_position();
			//            fe_goal = -60 * mel::DEG2RAD;
			//            ru_goal = ow[2].get_position();
			//        }
			//        else if (pred_label == 3) {
			//            // Radial Deviation
			//            ps_goal = ow[0].get_position();
			//            fe_goal = ow[1].get_position();
			//            ru_goal = 30 * mel::DEG2RAD;
			//        }
			//        else if (pred_label == 4) {
			//            // Ulnar Deviation
			//            ps_goal = ow[0].get_position();
			//            fe_goal = ow[1].get_position();
			//            ru_goal = -30 * mel::DEG2RAD;
			//        }
			//        else if (pred_label == 5) {
			//            // Pronation
			//            ps_goal = 80 * mel::DEG2RAD;
			//            fe_goal = ow[1].get_position();
			//            ru_goal = ow[2].get_position();
			//        }
			//        else if (pred_label == 6) {
			//            // Supination
			//            ps_goal = -80 * mel::DEG2RAD;
			//            fe_goal = ow[1].get_position();
			//            ru_goal = ow[2].get_position();
			//        }
			//        cooldown_clock.restart(); // restart so OpenWrist goal is only change once per half second
			//    }
		}
		//else {
		//    ps_goal = 0.0;
		//    fe_goal = 0.0;
		//    ru_goal = 0.0;
		//}

		//// set previous label
		//prev_pred_label = pred_label;

		//// set OpenWrist torques
		//ow[0].set_torque(pd0.move_to_hold(ps_goal, ow[0].get_position(),
		//    move_speed, ow[0].get_velocity(),
		//    0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
		//ow[0].add_torque(ow.compute_gravity_compensation(0));

		//ow[1].set_torque(pd1.move_to_hold(fe_goal, ow[1].get_position(),
		//    move_speed, ow[1].get_velocity(),
		//    0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
		//ow[0].add_torque(ow.compute_gravity_compensation(1));

		//ow[2].set_torque(pd2.move_to_hold(ru_goal, ow[2].get_position(),
		//    move_speed, ow[2].get_velocity(),
		//    0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
		//ow[0].add_torque(ow.compute_gravity_compensation(2));


		// write to MelShares
		ms_mes_env.write_data(mes.get_envelope());
		ms_mes_dm.write_data(mes.get_demean());
		ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });




		// check limits
		//if (ow.any_limit_exceeded()) {
		//    ctrlc == true;
		//}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			//ctrlc = true;
		}

		// update hardware
		//q8.update_output();

		// wait for remainder of sample period
		timer.wait();

	} // end control loop


	return;


}

MYOClassifier::~MYOClassifier()
{
}

int*** MYOClassifier::Parse(std::string filename) {

	std::ifstream       file(filename);

	int i = 0;
	int h = 0;

	int*** Training_Sets = new int**[7];

	for (CSVIterator row(file); row != CSVIterator(); ++row)
	{

		if (i == 0) {
			Training_Sets[h] = new int*[200];

		}

		Training_Sets[h][i] = new int[8];
		for (int j = 0; j < 8; j++) {

			Training_Sets[h][i][j] = std::stoi((*row)[j]);
			//std::cout << Training_Sets[h][i][j] << " ";

		}

		//std::cout << std::endl;
		i++;
		//prompt("Press Enter");
		if (i == 200) {
			i = 0;
			h++;
		}
	}



	//prompt("Press Enter");
	file.close();

	return Training_Sets;
}

