#include <MEL/Math/Functions.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Mechatronics/PositionSensor.hpp>
#include <MEL/Mechatronics/VelocitySensor.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Logging/Log.hpp>
#include "SparGlove.hpp"
#include <MEL/Devices/Windows/Keyboard.hpp>
#include "MYO_Classifier.hpp"




using namespace mel;
//==============================================================================
// MISC
//==============================================================================

// create global stop variable CTRL-C handler function


/*
//Thread function, open a thread class?
int MYOClassifier() {
while (sharedVariable != 1) {
// mutex.lock()
// change the variables
// mutex.unlock()
}
return 0;
}
*/

//==============================================================================
// MAIN
//==============================================================================


int GloveDriver(int MYO) {

	// register CTRL-C handler
	// make and parse console options
	/*Options options("spar_glove.exe", "Spar Glove demo");
	options.add_options()
	("o,home", "Homing")
	("h,help", "Prints this Help Message");*/
	//auto input = options.parse(argc, argv);

	// print help message if requested

	// create Spar Glove instance
	SparGlove sg;

	// enable Spar Glove
	prompt("Press ENTER to enable SPAR Glove");
	sg.enable();

	print("Enabled the Spar Glove");

	//sg.experiment1(stop);

	// disable spare glove
	sg.disable();

	return 0;
}

//	std::thread myThread(threadFunction);
//std::thread IntentDetection(MYOClassifier);


int main1() {
	std::vector <std::string> v;
	v = {};// "Fist_Training.csv", "PointTraining.csv", "ButtonTraining.csv", "ExtensionTraining.csv", "OKTraining.csv", "ThumbOppoTraining.csv"};

	//MYOClassifier mc = MYOClassifier(v);
	SparGlove sg;

	sg.on_enable();
	sg.zero_encoders();

	sg.on_enable();
	sg.start_myo();
	//sg.start_turning();
	

	return 0;

}

int main() {
	std::vector <std::string> v;
	v = { "PointTraining.csv", "ButtonTraining.csv", "ExtensionTraining.csv", "SCI_Fist_Training.csv" , "OKTraining.csv", "ThumbOppoTraining.csv", "ThumbsUpTraining.csv" };

	SparGlove sg;

	//sg.enable();

	MYOClassifier mc = MYOClassifier(v);

	return 0;
}

int main2() {
	Q8Usb q8;
	q8.open();
	q8.AO.set_disable_values(std::vector<Voltage>(8, 0.0)); // default is 0.0
	q8.AO.set_expire_values(std::vector<Voltage>(8, 0.0));  // default is 0.0
	q8.enable();

	q8.AO[0].set_value(0.9);

	Timer timer(hertz(1000));

	q8.watchdog.set_timeout(milliseconds(100));
	q8.watchdog.start();

	while (true) {
		q8.update_input();
		q8.watchdog.kick();
		if (Keyboard::is_key_pressed(Key::Space))
			break;
		q8.update_output();
		timer.wait();
	}

	q8.disable();
	q8.close();
	return 0;
}