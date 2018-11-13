#include <Spar_glove.hpp>

//==============================================================================
// MISC
//==============================================================================

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    if (event == CtrlEvent::CtrlC) {
        print("Ctrl+C Pressed!");
        stop = true;
    }
    return true;
}

//==============================================================================
// TUNNEL DEMO
//==============================================================================

Waveform tunnel_trajectory = Waveform(Waveform::Sin, seconds(2), 30.0 * DEG2RAD); //"Waveform" is a class, What is its use on either side of =?????????????????????????????
PdController tunnel_pd     = PdController(1.0, 0.01);

double tunnel(double position, double velocity, Time current_time) { //Seems to be less a tunnel than a path??????????????????????????????????????????
    double x_ref = tunnel_trajectory.evaluate(current_time);
    return tunnel_pd.calculate(x_ref, position, 0, velocity);
}




//==============================================================================
// MAIN
//==============================================================================

int sharedVariable = 0;
Mutex gMutex;

int threadFunction() {
	while (sharedVariable != 1) {
		// mutex.lock()
		// change the variables
		// mutex.unlock()
	}
	return 0;
}


int main(int argc, char* argv[]) {

	std::thread myThread(threadFunction);

	// mutex.lock()
	// change the variables
	// mutex.unlock()



    // register CTRL-C handler
    register_ctrl_handler(handler); 

    // make and parse console options
    Options options("spar_glove.exe", "Spar Glove demo");
    options.add_options()
    	("o,home", "Homing")
        ("h,help", "Prints this Help Message");
    auto input = options.parse(argc, argv);

    // print help message if requested
    if (input.count("h") > 0) {
        print(options.help());
        return 0;
    }


    // create Q8 USB
    Q8Usb q8;
    //q8.encoder.set_units_per_count(1/14155);
    q8.open();

    // create Spar Glove instance
	SparGlove sg(q8.DO[0], q8.AO[0], q8.AI[0], q8.encoder[0]);

    // enable Q8 Usb
    q8.enable();

    // enable Spar Glove
    prompt("Press ENTER to enable SPAR Glove");
    sg.enable();

	print("Enabled the Spar Glove");

	MelShare ms("ms_position");

    // create control loop timer
    Timer timer(hertz(1000));

    // enter control loop
    prompt("Press ENTER to start control loop");
    while (!stop) {

        // update hardware
        q8.update_input();        
		
		ms.write_data({ sg[0].get_position() });

        q8.update_output();

        // wait timer
        timer.wait(); 
    }

    return 0;
}
