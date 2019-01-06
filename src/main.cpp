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
ctrl_bool stop(false);
bool handler1(CtrlEvent event) {
    if (event == CtrlEvent::CtrlC) {
        print("Ctrl+C Pressed!");
        stop = true;
    }
    return true;
}

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
    register_ctrl_handler(handler1); 

//	std::thread myThread(threadFunction);

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


int main() {
	std::vector <std::string> v;
	v = { "Fist_Training.csv", };

	SparGlove sg;

	//sg.enable();

	MYOClassifier mc = MYOClassifier(v, sg);

	return 0;
}