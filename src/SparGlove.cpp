#include "SparGlove.hpp"
#include <MEL/Devices/Windows/Keyboard.hpp>

ctrl_bool g_stop(false);
bool handler1(CtrlEvent event) {
	if (event == CtrlEvent::CtrlC) {
		print("Ctrl+C Pressed!");
		g_stop = true;
	}
	return true;
}


SparGlove::SparGlove() :

	Robot("spar_glove"),
	q8_(),
	kt_{ -.00413, -.00413, .00413, .00413, -.00413, .00413, .00413 }, // mN * m / amp
	motor_cont_limits_{ 3.63, 3.63, 3.63, 3.63, 3.63, 3.63, 3.63 },  // amps
	motor_peak_limits_{ 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0 }, // amps
	motor_i2t_times_{ seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1) },
	eta_{ 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001 }, //gearbox ratio including screw pitch
	encoder_res_{ 512, 512, 512, 512, 512, 512, 512 }, //ticks/rev
	pos_limits_neg_{ 0,0,0,0,0,0,0 }, //m
	pos_limits_pos_{ .07, .07, .07, .07, .07, .07, .07 }, //m
	vel_limits_{ 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03 },
	joint_torque_limits{ 60, 60, 60, 60, 60, 60, 60 },
	kin_friction_{ 0,0,0,0,0,0,0 },
	pd_controllers_(7),
	encoders_(7)
{
	//motor_cont_limits_ = std::array<double, 7>({ 1,2,3,4,5,6,7 });
	//motor_peak_limits_[0] = 1;
	//motor_peak_limits_[1] = 2;
	// ...

	// Components
	amps_.reserve(7);
	amps_.emplace_back("amp1", Low, q8_.DO[0], 1.8, q8_.AO[0]); // THERE ARE MORE ARGS AVAILABLE!
	amps_.emplace_back("amp2", Low, q8_.DO[1], 1.8, q8_.AO[1]); // THERE ARE MORE ARGS AVAILABLE!
	amps_.emplace_back("amp3", Low, q8_.DO[2], 1.8, q8_.AO[2]);
	amps_.emplace_back("amp4", Low, q8_.DO[3], 1.8, q8_.AO[3]);
	amps_.emplace_back("amp5", Low, q8_.DO[4], 1.8, q8_.AO[4]);
	amps_.emplace_back("amp6", Low, q8_.DO[5], 1.8, q8_.AO[5]);
	amps_.emplace_back("amp7", Low, q8_.DO[6], 1.8, q8_.AO[6]);


	motors_.reserve(7);
	motors_.push_back(Motor("motor0", kt_[0], amps_[0]));
	motors_.push_back(Motor("motor1", kt_[1], amps_[1]));
	motors_.push_back(Motor("motor2", kt_[2], amps_[2]));
	motors_.push_back(Motor("motor3", kt_[3], amps_[3]));
	motors_.push_back(Motor("motor4", kt_[4], amps_[4]));
	motors_.push_back(Motor("motor5", kt_[5], amps_[5]));
	motors_.push_back(Motor("motor6", kt_[6], amps_[6]));
	//motors_.emplace_back("motor0", kt_[0], amps_[0]);
	// do 6 more times


	pd_controllers_[0] = PdController(0.2, 0.01);
	pd_controllers_[1] = PdController(0.2, 0.01);
	pd_controllers_[2] = PdController(0.2, 0.01);
	pd_controllers_[3] = PdController(0.2, 0.01);
	pd_controllers_[4] = PdController(0.2, 0.01);
	pd_controllers_[5] = PdController(0.2, 0.01);
	pd_controllers_[6] = PdController(0.2, 0.01);// do 6 more times

	for (std::size_t i = 0; i < 7; ++i) {

		encoders_[i] = q8_.encoder[i];

		Joint joint(
			"Spar_joint_" + std::to_string(i),
			&motors_[i],
			eta_[i],
			&encoders_[i],
			eta_[i],
			&encoders_[i],
			eta_[i],
			std::array<double, 2>({ pos_limits_neg_[i], pos_limits_pos_[i] }),
			vel_limits_[i],
			joint_torque_limits[i]
		);
		add_joint(joint);
	}

}
//
//SparGlove::SparGlove(const SparGlove&):
//	Robot("spar_glove"),
//	q8_(),
//	kt_{ 1,2,3,4,5,6,7 },
//	motor_cont_limits_{ 1,2,3,4,5,6,7 },  //amps
//	motor_peak_limits_{ 1,2,3,4,5,6,7 }, //amps
//	motor_i2t_times_{ seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1) },
//	eta_{ 1,2,3,4,5,6,7 },
//	encoder_res_{ 1,2,3,4,5,6,7 },
//	pos_limits_neg_{ 1,2,3,4,5,6,7 },
//	pos_limits_pos_{ 1,2,3,4,5,6,7 },
//	vel_limits_{ 1,2,3,4,5,6,7 },
//	joint_torque_limits{ 1,2,3,4,5,6,7 },
//	kin_friction_{ 1,2,3,4,5,6,7 },
//	pd_controllers_(7),
//	encoders_(7)
//{
//	register_ctrl_handler(handler1);
//
//}

SparGlove::~SparGlove() {
	if (is_enabled())
		get_joint(0).set_torque(0.0);
	on_disable();
}

void SparGlove::zero_encoders() {
	q8_.enable();
	std::vector<int32> encoder_offsets = { 0, 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < 7; i++) {
		encoders_[i].reset_count(encoder_offsets[i]);
	}
	q8_.disable();
}


void SparGlove::set_pred_label(const std::size_t pose) {

	mel::Lock lock(mtx);

	//mtx.lock();
	this->pred_label = pose;
	//mtx.unlock();
}
//
//size_t SparGlove::get_pred_label() {
//	mel::Lock lock(mtx);
//
//	//mtx.lock();
//	std::size_t result = this->pred_label;
//	//mtx.unlock();
//	return result;
//	//return 0;
//}

void SparGlove::notify(const std::size_t pose) {
	//Functions that need to know about pred_label
	//Experiment1
	//experiment1(std::atomic_bool( false ), pred_label);
	set_pred_label(pose);
	//GloveDriver
}

void SparGlove::start_myo() {
	//Timer timer(hertz(1000));

	//MelShare ms("motor_position");
	//std::vector<double> to_ms_data(4);

	//// enter control loop
	////prompt("Press ENTER ldskfsldfjdto start control loop");
	//
	//q8_.watchdog.set_timeout(milliseconds(100));

	//q8_.watchdog.start();

	///////////////////////////////////
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
	// make MelShares
	MelShare ms_Torque("Torque");

	MelShare ms_fake_Torque("fakeTorque");
	MelShare ms_Pos_Ref("Pos_Ref");
	MelShare ms_Pos_Act("Pos_Act");

	// initialize testing conditions
	Time Ts = milliseconds(1); // sample period
	std::size_t num_classes = 7; // number of active classes
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };
	Time mes_active_capture_period = seconds(0.2);
	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
	mes.resize_buffer(mes_active_capture_window_size);
	std::size_t pred_label = 0;
	std::size_t prev_pred_label = 0;
	std::size_t fake_pred_label = 0;

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

	// prompt the user for input
	print("Press 'A + target #' to add training data for one target.");
	print("Press 'C + target #' to clear training data for one target.");
	print("Number of targets/classes is:");
	print(num_classes);
	print("Press 'T' to train classifier and begin real-time classification.");
	print("Press 'R' to run the Sparglove");
	print("Press 'S' to stop the Sparglove");
	print("Press 'Escape' to exit.");

	// enable hardware
	q8_.watchdog.start();
	myo.enable();

	while (!g_stop) {

		// update hardware
		q8_.watchdog.kick();
		q8_.update_input();

		// update all DAQ input channels
		myo.update();

		// emg signal processing
		mes.update_and_buffer();

		// predict state
		if (dir_classifier.update(mes.get_demean())) {
			pred_label = dir_classifier.get_class();
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

		//fake prediction
		for (std::size_t k = 0; k < active_keys.size(); ++k) {
			if (Keyboard::is_key_pressed(active_keys[k])) {
				if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
					fake_pred_label = k;
					std::cout << fake_pred_label << std::endl;
					keypress_refract_clock.restart();
				}
			}
		}

		//if (run) {
		//	if (cooldown_clock.get_elapsed_time() > seconds(0.5)) {
		//		if (pred_label == 0) {
		//			// Rest
		//			ps_goal = ow[0].get_position();
		//			fe_goal = ow[1].get_position();
		//			ru_goal = ow[2].get_position();
		//		}
		//		if (pred_label == 1) {
		//			// Flexion
		//			ps_goal = ow[0].get_position();
		//			fe_goal = 60 * mel::DEG2RAD;
		//			ru_goal = ow[2].get_position();
		//		}
		//		else if (pred_label == 2) {
		//			// Extension
		//			ps_goal = ow[0].get_position();
		//			fe_goal = -60 * mel::DEG2RAD;
		//			ru_goal = ow[2].get_position();
		//		}
		//		else if (pred_label == 3) {
		//			// Radial Deviation
		//			ps_goal = ow[0].get_position();
		//			fe_goal = ow[1].get_position();
		//			ru_goal = 30 * mel::DEG2RAD;
		//		}
		//		else if (pred_label == 4) {
		//			// Ulnar Deviation
		//			ps_goal = ow[0].get_position();
		//			fe_goal = ow[1].get_position();
		//			ru_goal = -30 * mel::DEG2RAD;
		//		}
		//		else if (pred_label == 5) {
		//			// Pronation
		//			ps_goal = 80 * mel::DEG2RAD;
		//			fe_goal = ow[1].get_position();
		//			ru_goal = ow[2].get_position();
		//		}
		//		else if (pred_label == 6) {
		//			// Supination
		//			ps_goal = -80 * mel::DEG2RAD;
		//			fe_goal = ow[1].get_position();
		//			ru_goal = ow[2].get_position();
		//		}
		//cooldown_clock.restart(); // restart so OpenWrist goal is only change once per half second
		//	}
		//}
		//else {
		//	ps_goal = 0.0;
		//	fe_goal = 0.0;
		//	ru_goal = 0.0;
		//}

		// set previous label
		prev_pred_label = pred_label;

		// set OpenWrist torques
		/*ow[0].set_torque(pd0.move_to_hold(ps_goal, ow[0].get_position(),
		move_speed, ow[0].get_velocity(),
		0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
		ow[0].add_torque(ow.compute_gravity_compensation(0));

		ow[1].set_torque(pd1.move_to_hold(fe_goal, ow[1].get_position(),
		move_speed, ow[1].get_velocity(),
		0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
		ow[0].add_torque(ow.compute_gravity_compensation(1));

		ow[2].set_torque(pd2.move_to_hold(ru_goal, ow[2].get_position(),
		move_speed, ow[2].get_velocity(),
		0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
		ow[0].add_torque(ow.compute_gravity_compensation(2));
		*/

		// write to MelShares
		ms_mes_env.write_data(mes.get_envelope());
		ms_mes_dm.write_data(mes.get_demean());
		ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });

		//double pos_ref = get_joint(0).get_position();
		//double pos_act = get_joint(4).get_position();
		//double vel_act = get_joint(4).get_velocity();
		double torque = pred_label * 0.1;
		//double torque2 = pd_controllers_[4].calculate(pos_ref, pos_act, 0, vel_act);
		double torque3 = fake_pred_label * 0.1; 
		double sat = .9;

		torque = saturate(torque, sat);

		get_joint(4).set_torque(torque);

		// write to MelShares
		ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
		ms_fake_Torque.write_data({ (torque3) });
		ms_Torque.write_data({ (torque) });
		//ms_Pos_Ref.write_data({ (pos_ref) });
		//ms_Pos_Act.write_data({ (pos_act) });

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			g_stop = true;
		}

		// update hardware
		q8_.update_output();

		// wait for remainder of sample period
		timer.wait();

	} // end control loop

	  //////////////////////////////////
}
/*
while (!g_stop && !Keyboard::is_key_pressed(Key::Escape)) {

//MelShare Spar_pred1_label("Spar_pred2_label");
//Spar_pred1_label.write_data({ (double)((signed)(get_pred_label())) });
//{
//Lock lock(mtx);
//	std::cout << "But SparGlove thinks pred_label is" << std::endl;

////Prove that we can see pred_label

//	std::cout << get_pred_label() << std::endl;

//	print("SparGlove Main loop");
//}
//std::cout << "MYO_Classifier thinks pred_label is" << std::endl;

//std::cout << pred_label << std::endl;


// update hardware
q8_.update_input();
q8_.watchdog.kick();


double torque = 1.0;

switch (get_pred_label()) {
case (size_t)0:
torque = 0.0;
break;
case (size_t)1:
torque = 0.1;
break;
case (size_t)2:
torque = 0.2;
break;
case (size_t)3:
torque = 0.3;
break;
case (size_t)4:
torque = 0.4;
break;
case (size_t)5:
torque = 0.5;
break;
case (size_t)6:
torque = 0.6;
break;

default:
torque = 0.9;

}

//	std::array<double, 7> sat_torques = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; // temporary saturation torques
double sat = 5.0;

// experiment 1 code
double pos_ref = get_joint(1).get_position();
double pos_act = get_joint(0).get_position();
double vel_act = get_joint(0).get_velocity();
double torque2 = pd_controllers_[0].calculate(pos_ref, pos_act, 0, vel_act);

double nothing = 0.0;
//		torque = saturate(torque, sat);
torque2 = saturate(torque2, sat);

get_joint(3).set_torque(torque);
get_joint(0).set_torque(torque);

to_ms_data[0] = pos_ref;
to_ms_data[1] = pos_act;
to_ms_data[2] = nothing;  // vel_act;
to_ms_data[3] = torque2;

ms.write_data(to_ms_data);

//ms.write_data({ get_joint(0).get_position() });
//ms.write_data({ get_joint(1).get_position() });

// update output
q8_.update_output();


// wait timer
timer.wait();
}
print("STARTED ENDED!!");
};
*/

void SparGlove::start_turning() {
	Time Ts = milliseconds(1); // sample period
	std::size_t pred_label = 0;
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };

	// construct clock to regulate interaction
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.2);

	// make MelShares
	MelShare ms_pred_label("pred_label");
	MelShare ms_Torque("Torque");
	MelShare ms_Pos_Ref("Pos_Ref");
	MelShare ms_Pos_Act("Pos_Act");

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);

	q8_.watchdog.start();

	while (!g_stop) {

		// update hardware
		q8_.watchdog.kick();
		q8_.update_input();


		//fake prediction
		for (std::size_t k = 0; k < active_keys.size(); ++k) {
			if (Keyboard::is_key_pressed(active_keys[k])) {
				if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
					pred_label = k;
					std::cout << pred_label << std::endl;
					keypress_refract_clock.restart();
				}
			}
		}



		double pos_ref = get_joint(0).get_position();
		double pos_act = get_joint(4).get_position();
		double vel_act = get_joint(4).get_velocity();
		double torque = pred_label * 0.1;
		double torque2 = pd_controllers_[4].calculate(pos_ref, pos_act, 0, vel_act);  
		double sat = .1;

		torque2 = saturate(torque2, sat);

		get_joint(4).set_torque(torque);


		/*if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
		if (Keyboard::is_key_pressed(Key::Num1)) {

		std::cout << "Yep we're in the loop1" << std::endl;
		pred_label = 0;
		}
		else if (Keyboard::is_key_pressed(Key::Num2)) {

		std::cout << "Yep we're in the loop2" << std::endl;
		pred_label = 1;
		}
		else if (Keyboard::is_key_pressed(Key::Num3)) {
		pred_label = 2;
		}
		else if (Keyboard::is_key_pressed(Key::Num4)) {
		pred_label = 3;
		}
		else if (Keyboard::is_key_pressed(Key::Num5)) {
		pred_label = 4;
		}
		else if (Keyboard::is_key_pressed(Key::Num6)) {
		pred_label = 5;
		}
		else if (Keyboard::is_key_pressed(Key::Num7)) {
		pred_label = 6;
		}
		keypress_refract_clock.restart();
		}*/
		//if (true) {
		//	if (cooldown_clock.get_elapsed_time() > seconds(0.5)) {
		//		if (pred_label == 0) {
		//			// Rest
		//			ps_goal = ow[0].get_position();
		//			fe_goal = ow[1].get_position();
		//			ru_goal = ow[2].get_position();
		//		}
		//		if (pred_label == 1) {
		//			// Flexion
		//			ps_goal = ow[0].get_position();
		//			fe_goal = 60 * mel::DEG2RAD;
		//			ru_goal = ow[2].get_position();
		//		}
		//		else if (pred_label == 2) {
		//			// Extension
		//			ps_goal = ow[0].get_position();
		//			fe_goal = -60 * mel::DEG2RAD;
		//			ru_goal = ow[2].get_position();
		//		}
		//		else if (pred_label == 3) {
		//			// Radial Deviation
		//			ps_goal = ow[0].get_position();
		//			fe_goal = ow[1].get_position();
		//			ru_goal = 30 * mel::DEG2RAD;
		//		}
		//		else if (pred_label == 4) {
		//			// Ulnar Deviation
		//			ps_goal = ow[0].get_position();
		//			fe_goal = ow[1].get_position();
		//			ru_goal = -30 * mel::DEG2RAD;
		//		}
		//		else if (pred_label == 5) {
		//			// Pronation
		//			ps_goal = 80 * mel::DEG2RAD;
		//			fe_goal = ow[1].get_position();
		//			ru_goal = ow[2].get_position();
		//		}
		//		else if (pred_label == 6) {
		//			// Supination
		//			ps_goal = -80 * mel::DEG2RAD;
		//			fe_goal = ow[1].get_position();
		//			ru_goal = ow[2].get_position();
		//		}
		//		cooldown_clock.restart(); // restart so OpenWrist goal is only change once per half second
		//	}
		//}
		//else {
		//	ps_goal = 0.0;
		//	fe_goal = 0.0;
		//	ru_goal = 0.0;
		//}

		// write to MelShares
		ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
		ms_Torque.write_data({ (torque2) });
		ms_Pos_Ref.write_data({ (pos_ref) });
		ms_Pos_Act.write_data({ (pos_act) });
		

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			g_stop = true;
		}

		// update hardware
		q8_.update_output();

		// wait for remainder of sample period
		timer.wait();

	}


}

/// Overrides the default Robot::enable function with some custom logic
bool SparGlove::on_enable() {
	std::cout << "SparGlove is TOO enabled" << std::endl;
	q8_.open();
	mel::prompt("Press ENTER to open and enable Q8 USB.");
	q8_.AO.set_disable_values(std::vector<Voltage>(8, 0.0)); // default is 0.0
	q8_.AO.set_expire_values(std::vector<Voltage>(8, 0.0));  // default is 0.0

	q8_.DO.set_disable_values(std::vector<Logic>(8, High)); // default is 0.0
	q8_.DO.set_expire_values(std::vector<Logic>(8, High));  // default is 0.0

	q8_.enable();

	//std::thread first(&SparGlove::start, this);
	//first.detach();

	return true;


	//if (q8_.open() && q8_.enable()){// && Robot::on_enable()) {
	//	for (int i = 0; i < 8; i++) {
	//		q8_.DO[i].set_disable_value(High);
	//		q8_.DO[i].set_expire_value(High);
	//	}

	//	return true;
	//}
	//else
	//	return false;	
}

/// Overrides the default Robot::enable function with some custom logic
bool SparGlove::on_disable() {


	if (Robot::on_disable() && q8_.disable() && q8_.close())
	{


		return true;
	}
	else
		return false;

}