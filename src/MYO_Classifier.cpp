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

MYOClassifier::MYOClassifier(std::vector<std::string> training_files) { //, SparGlove& sg) {
	
	// handle inputs
	std::vector<uint32> emg_channel_numbers = { 0,1,2,3,4,5,6,7 };

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

	int pose = 0;

	int*** read_in_object = new int**[7];

	for (std::vector<std::string>::iterator it = training_files.begin(); it != training_files.end(); ++it) {
		Parse(*it, read_in_object);
		
		for (int k = 0; k <= 6; k++) {
			std::vector<std::vector<double>> training_set;

			for (int L = 0; L < 200; L++) {
				std::vector<double> row;

				for (int m = 0; m < 8; m++) {
					
					row.push_back(read_in_object[k][L][m]);  //converts integers in input file to double precision which RealTimeClassifier expects
					
				}
				training_set.push_back(row);
				std::cout << row << std::endl;
			}
			dir_classifier.add_training_data(pose, training_set);

			//std::cout << training_set.size() << std::endl;

			if (dir_classifier.add_training_data(pose, training_set)) {
				LOG(Info) << "Added active data for target " + stringify(pose + 1) + ".";
			}
			
		}
		pose++;
		
	}

	for (int i = 0; i < 7; i++) {
		for (int j = 0; j < 200; j++) {
			delete read_in_object[i][j];
		}
		delete read_in_object[i];
	}
	delete read_in_object;

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

	//myo.enable();

	while (true) {
		
		// update all DAQ input channels
		//myo.update();

		// emg signal processing
		mes.update_and_buffer();
		
		// predict state
		if (dir_classifier.update(mes.get_demean())) {
			pred_label = dir_classifier.get_class();
			//sg.notify(pred_label);
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
					dir_classifier.save();
				}
				keypress_refract_clock.restart();
			}
		}

		if (Keyboard::is_key_pressed(Key::U)) {
			if (dir_classifier.is_trained()) {
				
				std::string file_to_parse;
				std::cout << "Which file would you like to classify?" << std::endl;
				std::getline(std::cin, file_to_parse);
				pred_label = 9;
				std::cout << pred_label << std::endl;

				int*** data_to_classify = new int**[1];

				//for (std::vector<std::string>::iterator it = file_to_parse.begin(); it != file_to_parse.end(); ++it) {
					Parse(file_to_parse, data_to_classify);

						for (int L = 0; L < 200; L++) {
							std::vector<double> row;

							for (int m = 0; m < 8; m++) {

								row.push_back(data_to_classify[0][L][m]);  //converts integers in input file to double precision which RealTimeClassifier expects
							
							}
							//classification_set.push_back(row);
							dir_classifier.update(row);
						}

						//dir_classifier.update(classification_set); //needs a for loop to add each row of the classification set as a live stream.  This should match what "update" is looking for
						pred_label = dir_classifier.get_class();

						/*
						std::cout << classification_set.size() << std::endl;

						Butterworth filter(8, hertz(1000), .05);
					    
						std::vector<double> butt_class;

						for (int i = 0; i < 200; i++) {
							butt_class[i] = filter.update(classification_set[1][i]);
						}
*/
	//					if (dir_classifier.update(butt_class)) {
							std::cout << "And the winner is..." << std::endl;
							//int bucket = dir_classifier.get_class();
							std::cout << pred_label << std::endl;

							std::cout << "We've made a prediction!" << std::endl; 
							
							for (int i = 0; i < 1; i++) {
								for (int j = 0; j < 200; j++) {
									delete data_to_classify[i][j];
								}
								delete data_to_classify[i];
							}
							delete data_to_classify;
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

	return;
}

MYOClassifier::~MYOClassifier()
{
}

int*** MYOClassifier::Parse(std::string filename, int*** shell_array) {

	std::ifstream       file(filename);

	int i = 0;
	int h = 0;

	for (CSVIterator row(file); row != CSVIterator(); ++row)
	{

		if (i == 0) {
			shell_array[h] = new int*[200];

		}

		shell_array[h][i] = new int[8];
		for (int j = 0; j < 8; j++) {
			shell_array[h][i][j] = std::stoi((*row)[j]);
		}

		i++;
		//prompt("Press Enter");
		if (i == 200) {
			i = 0;
			h++;
		}
	}

	//prompt("Press Enter");
	file.close();


	//return Training_Sets;
	
}

