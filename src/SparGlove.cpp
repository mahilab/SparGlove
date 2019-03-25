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
	kt_{ .0109, .0109, .0109, .0109, .0109, .0109, .0109 }, // N * m / amp
	motor_cont_limits_{ 3.63, 3.63, 3.63, 3.63, 3.63, 3.63, 3.63 },  // amps
	motor_peak_limits_{ 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0 }, // amps
	motor_i2t_times_{ seconds(1.116), seconds(1.116), seconds(1.116), seconds(1.116), seconds(1.116), seconds(1.116), seconds(1.116) },
	//eta_{ 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001, 4.4*3.1831*.0001 }, //gearbox ratio including screw pitch
	eta_{ 1 / 4.4 * 2 / (2 * PI), 1 / 4.4 * 2 / (2 * PI), 1 / 4.4 * 2 / (2 * PI), 1 / 4.4 * 2 / (2 * PI), 1 / 4.4 * 2 / (2 * PI), 1 / 4.4 * 2 / (2 * PI), 1 / 4.4 * 2 / (2 * PI) },
	encoder_res_{ 512, 512, 512, 512, 512, 512, 512 }, //ticks/rev
	pos_limits_neg_{ 0, 0, 0, 0, 0, 0, 0 }, //mm
	pos_limits_pos_{ 75, 75, 75, 75, 75, 75, 75 }, //mm, determined experimentally
	vel_limits_{ 35, 35, 35, 35, 35, 35, 35 }, //Motor should complete its stroke in ~2 seconds, which is intentionally made slow. Can be changed if necessary
	joint_torque_limits{ 60, 60, 60, 60, 60, 60, 60 }, //limits set intentionally high, current limits will be the driving limiter////////////////////////////
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
		encoders_[i].set_units_per_count(2 * PI / encoder_res_[i]);

		Joint joint(
			"Spar_joint_" + std::to_string(i),
			&motors_[i],
			//eta_[i],
			&encoders_[i],
			//eta_[i],
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

//void SparGlove::zero_encoders() {
//	q8_.enable();
//	std::vector<int32> encoder_offsets = { 0, 0, 0, 0, 0, 0, 0 };
//	for (int i = 0; i < 7; i++) {
//		encoders_[i].reset_count(encoder_offsets[i]);
//	}
//	q8_.disable();
//}
//
//
//void SparGlove::set_pred_label(const std::size_t pose) {
//
//	mel::Lock lock(mtx);
//
//	//mtx.lock();
//	this->pred_label = pose;
//	//mtx.unlock();
//}
////
////size_t SparGlove::get_pred_label() {
////	mel::Lock lock(mtx);
////
////	//mtx.lock();
////	std::size_t result = this->pred_label;
////	//mtx.unlock();
////	return result;
////	//return 0;
////}
//
//void SparGlove::notify(const std::size_t pose) {
//	//Functions that need to know about pred_label
//	//Experiment1
//	//experiment1(std::atomic_bool( false ), pred_label);
//	set_pred_label(pose);
//	//GloveDriver
//}
//
//void SparGlove::start_myo() {
//	//Timer timer(hertz(1000));
//
//	//MelShare ms("motor_position");
//	//std::vector<double> to_ms_data(4);
//
//	//// enter control loop
//	////prompt("Press ENTER ldskfsldfjdto start control loop");
//	//
//	//q8_.watchdog.set_timeout(milliseconds(100));
//
//	//q8_.watchdog.start();
//
//	///////////////////////////////////
//	// handle inputs
//	std::vector<uint32> emg_channel_numbers = { 0,1,2,3,4,5,6,7 };
//
//	mel::MyoBand myo("my_myo");
//
//	// construct array of Myoelectric Signals
//	MesArray mes(myo.get_channels(emg_channel_numbers));
//
//	// make MelShares
//	MelShare ms_mes_env("mes_env");
//	MelShare ms_mes_dm("mes_dm");
//	MelShare ms_pred_label("pred_label");
//
//	// make MelShares
//	MelShare ms_Amps("amps");
//	MelShare ms_Ref("ref");
//	MelShare ms_Ref2("ref2");
//	MelShare ms_Pos_Act("Pos_Act");
//	MelShare ms_Pos_Act2("Pos_Act2");
//
//	// initialize testing conditions
//	Time Ts = milliseconds(1); // sample period
//	std::size_t num_classes = 7; // number of active classes
//	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };
//	Time mes_active_capture_period = seconds(0.2);
//	std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
//	mes.resize_buffer(mes_active_capture_window_size);
//	std::size_t pred_label = 0;
//	std::size_t prev_pred_label = 0;
//	std::size_t fake_pred_label = 0;
//
//	// initialize classifier
//	bool RMS = true;
//	bool MAV = false;
//	bool WL = false;
//	bool ZC = false;
//	bool SSC = false;
//	bool AR1 = true;
//	bool AR2 = false;
//	bool AR3 = false;
//	bool AR4 = false;
//
//	EmgDirClassifier dir_classifier(num_classes, emg_channel_numbers.size(), Ts, RMS, MAV, WL, ZC, SSC, AR1, AR2, AR3, AR4, seconds(0.4), seconds(0.2), seconds(0.3));
//
//	bool run = false;
//	Clock cooldown_clock;
//
//	// construct clock to regulate interaction
//	Clock keypress_refract_clock;
//	Time keypress_refract_time = seconds((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds());
//
//	// construct timer in hybrid mode to avoid using 100% CPU
//	Timer timer(Ts, Timer::Hybrid);
//	timer.set_acceptable_miss_rate(0.3);
//
//	// prompt the user for input
//	print("Press 'A + target #' to add training data for one target.");
//	print("Press 'C + target #' to clear training data for one target.");
//	print("Number of targets/classes is:");
//	print(num_classes);
//	print("Press 'T' to train classifier and begin real-time classification.");
//	print("Press 'R' to run the Sparglove");
//	print("Press 'S' to stop the Sparglove");
//	print("Press 'Escape' to exit.");
//
//	// enable hardware
//	//q8_.watchdog.start();
//	myo.enable();
//	double ref = 0.0;
//	double ref2 = 0.0;
//	int case0 = 0;
//	int case1 = 0;
//	int case2 = 0;
//	int case3 = 0;
//	int case4 = 0;
//	int case5 = 0;
//	int case6 = 0;
//
//	double setpoint_0 = 0.0;
//	double setpoint_1 = 0.0;
//	double setpoint_2 = 0.0;
//	double setpoint_3 = 0.0;
//	double setpoint_4 = 0.0;
//	double setpoint_5 = 0.0;
//	double setpoint_6 = 0.0;
//
//	while (!g_stop) {
//
//		// update hardware
//		//q8_.watchdog.kick();
//		q8_.update_input();
//
//		// update all DAQ input channels
//		myo.update();
//
//		// emg signal processing
//		mes.update_and_buffer();
//
//		// predict state
//		if (dir_classifier.update(mes.get_demean())) {
//			pred_label = dir_classifier.get_class();
//		}
//
//		// clear active data
//		for (std::size_t k = 0; k < num_classes; ++k) {
//			if (Keyboard::are_all_keys_pressed({ Key::C, active_keys[k] })) {
//				if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//					if (dir_classifier.clear_training_data(k)) {
//						LOG(Info) << "Cleared active data for target " + stringify(k + 1) + ".";
//					}
//					keypress_refract_clock.restart();
//				}
//			}
//		}
//
//		// capture active data
//		for (std::size_t k = 0; k < num_classes; ++k) {
//			if (Keyboard::are_all_keys_pressed({ Key::A, active_keys[k] })) {
//				if (mes.is_buffer_full()) {
//					if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//						if (dir_classifier.add_training_data(k, mes.get_dm_buffer_data(mes_active_capture_window_size))) {
//							LOG(Info) << "Added active data for target " + stringify(k + 1) + ".";
//						}
//						keypress_refract_clock.restart();
//					}
//				}
//			}
//		}
//
//		// train the active/rest classifiers
//		if (Keyboard::is_key_pressed(Key::T)) {
//			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//				if (dir_classifier.train()) {
//					LOG(Info) << "Trained new active/rest classifier based on given data.";
//					std::string directory_ = "C:\\Git\\SparGlove\\bin\\Release";
//					//dir_classifier.save("Current_Classifier1.csv", directory_);
//					dir_classifier.save("real_time_multi_classifier", "my_files", true);
//				}
//				keypress_refract_clock.restart();
//			}
//		}
//
//		if (Keyboard::is_key_pressed(Key::L)) {
//			if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//				std::string directory_ = "C:\\Git\\SparGlove\\bin\\Release";
//				dir_classifier.load("Current_Classifier.csv", directory_);
//				std::cout << "Loaded the most recent dir_classifier" << std::endl;
//				keypress_refract_clock.restart();
//			}
//		}
//
//		// set OpenWrist run state
//		if (Keyboard::is_key_pressed(Key::R) && keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//			LOG(Info) << "OpenWrist Running";
//			run = true;
//			keypress_refract_clock.restart();
//		}
//		else if (Keyboard::is_key_pressed(Key::S) && keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//			LOG(Info) << "OpenWrist Stopped";
//			run = false;
//			keypress_refract_clock.restart();
//		}
//
//		//fake prediction
//		for (std::size_t k = 0; k < active_keys.size(); ++k) {
//			if (Keyboard::is_key_pressed(active_keys[k])) {
//				if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//					fake_pred_label = k;
//					std::cout << fake_pred_label << std::endl;
//					keypress_refract_clock.restart();
//				}
//			}
//		}
//
//		// set previous label
//		prev_pred_label = pred_label;
//
//		// write to MelShares
//		ms_mes_env.write_data(mes.get_envelope());
//		ms_mes_dm.write_data(mes.get_demean());
//		ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
//
//		//double pos_ref = get_joint(0).get_position();
//		//double pos_act = get_joint(4).get_position();
//		//double vel_act = get_joint(4).get_velocity();
//		double torque = pred_label * 0.1;
//		//double torque2 = pd_controllers_[4].calculate(pos_ref, pos_act, 0, vel_act);
//		//double torque3 = fake_pred_label * 0.1;
//		//double sat = .9;
//
//		//torque = saturate(torque, sat);
//
//		//get_joint(4).set_torque(torque);
//
//
//
//		/////////////////////////////////////////////////////////////
//
//
//
//
//		double act_0 = get_joint(0).get_position();
//		double act_1 = get_joint(1).get_position();
//		double act_2 = get_joint(2).get_position();
//		double act_3 = get_joint(3).get_position();
//		double act_4 = get_joint(4).get_position();
//		double act_5 = get_joint(5).get_position();
//		double act_6 = get_joint(6).get_position();
//
//		double vel_0 = get_joint(0).get_velocity();
//		double vel_1 = get_joint(1).get_velocity();
//		double vel_2 = get_joint(2).get_velocity();
//		double vel_3 = get_joint(3).get_velocity();
//		double vel_4 = get_joint(4).get_velocity();
//		double vel_5 = get_joint(5).get_velocity();
//		double vel_6 = get_joint(6).get_velocity();
//
//		if (Keyboard::is_key_pressed(Key::E)) {
//			motors_[0].enable();
//			motors_[1].enable();
//		}
//		else if (Keyboard::is_key_pressed(Key::D)) {
//			motors_[0].disable();
//			motors_[1].disable();
//		}
//
//
//
//		/*int fake = 0;
//
//		if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//			if (Keyboard::is_key_pressed(Key::Num1)) {
//
//				std::cout << "Yep we're in the loop1" << std::endl;
//				fake = 0;
//			}
//			else if (Keyboard::is_key_pressed(Key::Num2)) {
//
//				std::cout << "Yep we're in the loop2" << std::endl;
//				fake = 1;
//			}
//			else if (Keyboard::is_key_pressed(Key::Num3)) {
//				fake = 2;
//			}
//			else if (Keyboard::is_key_pressed(Key::Num4)) {
//				fake = 3;
//			}
//			else if (Keyboard::is_key_pressed(Key::Num5)) {
//				fake = 4;
//			}
//			else if (Keyboard::is_key_pressed(Key::Num6)) {
//				fake = 5;
//			}
//			else if (Keyboard::is_key_pressed(Key::Num7)) {
//				fake = 6;
//			}
//			keypress_refract_clock.restart();
//		}*/
//
//		switch (fake_pred_label) {
//		case (size_t)0:
//			case0++;
//			torque = 0.2;
//
//			if (case0 > 10) {
//
//				setpoint_0 = 9;
//				setpoint_1 = 0;
//				setpoint_2 = 0;
//				setpoint_3 = 0;
//				setpoint_4 = 0;
//				setpoint_5 = 0;
//				setpoint_6 = 0;
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)1:
//			ref += 0.01;
//			ref2 += 0.01;
//			torque = 0.2;
//			case1++;
//			torque = 0.2;
//
//			if (case1 > 10) {
//
//				setpoint_0 = 0;
//				setpoint_1 = 9;
//				setpoint_2 = 0;
//				setpoint_3 = 0;
//				setpoint_4 = 0;
//				setpoint_5 = 0;
//				setpoint_6 = 0;
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			break;
//		case (size_t)2:
//			ref -= 0.01;
//			ref2 -= 0.01;
//			torque = 0.2;
//			case0++;
//			torque = 0.2;
//
//			if (case2 > 10) {
//
//				setpoint_0 = 0;
//				setpoint_1 = 0;
//				setpoint_2 = 9;
//				setpoint_3 = 0;
//				setpoint_4 = 0;
//				setpoint_5 = 0;
//				setpoint_6 = 0;
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)3:
//			ref += 0.01;
//			torque = 0.3;
//			case0++;
//			torque = 0.2;
//
//			if (case3 > 10) {
//
//				setpoint_0 = 0;
//				setpoint_1 = 0;
//				setpoint_2 = 0;
//				setpoint_3 = 9;
//				setpoint_4 = 0;
//				setpoint_5 = 0;
//				setpoint_6 = 0;
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)4:
//			ref2 += 0.01;
//			torque = 0.4;
//			case0++;
//			torque = 0.2;
//
//			if (case4 > 10) {
//
//				setpoint_0 = 0;
//				setpoint_1 = 0;
//				setpoint_2 = 0;
//				setpoint_3 = 0;
//				setpoint_4 = 9;
//				setpoint_5 = 0;
//				setpoint_6 = 0;
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)5:
//			ref -= 0.01;
//			torque = 0.2;
//			case0++;
//			torque = 0.2;
//
//			if (case5 > 10) {
//
//				setpoint_0 = 0;
//				setpoint_1 = 0;
//				setpoint_2 = 0;
//				setpoint_3 = 0;
//				setpoint_4 = 0;
//				setpoint_5 = 9;
//				setpoint_6 = 0;
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)6:
//			ref2 -= 0.01;
//			torque = 0.6;
//			case0++;
//			torque = 0.2;
//
//			if (case6 > 10) {
//
//				setpoint_0 = 0;
//				setpoint_1 = 0;
//				setpoint_2 = 0;
//				setpoint_3 = 0;
//				setpoint_4 = 0;
//				setpoint_5 = 0;
//				setpoint_6 = 9;
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//
//		default:
//			torque = 0.9;
//
//		}
//
//		double amps_0 = pd_controllers_[0].calculate(setpoint_0, act_0, 0, vel_0);
//		double amps_1 = pd_controllers_[1].calculate(setpoint_1, act_1, 0, vel_1);
//		double amps_2 = pd_controllers_[2].calculate(setpoint_2, act_2, 0, vel_2);
//		double amps_3 = pd_controllers_[3].calculate(setpoint_3, act_3, 0, vel_3);
//		double amps_4 = pd_controllers_[4].calculate(setpoint_4, act_4, 0, vel_4);
//		double amps_5 = pd_controllers_[5].calculate(setpoint_5, act_5, 0, vel_5);
//		double amps_6 = pd_controllers_[6].calculate(setpoint_6, act_6, 0, vel_6);
//
//
//		amps_0 = -saturate(amps_0, -0.17, 0.17);
//		amps_1 = -saturate(amps_1, -0.17, 0.17);
//		amps_2 = -saturate(amps_2, -0.17, 0.17);
//		amps_3 = -saturate(amps_3, -0.17, 0.17);
//		amps_4 = -saturate(amps_4, -0.17, 0.17);
//		amps_5 = -saturate(amps_5, -0.17, 0.17);
//		amps_6 = -saturate(amps_6, -0.17, 0.17);
//
//		if (std::abs(amps_0) > 0.18) {
//			g_stop = true;
//		}
//
//		// write to MelShares
//		ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
//		ms_Ref.write_data({ (ref) });
//		ms_Ref2.write_data({ (ref2) });
//		ms_Amps.write_data({ (amps_0) });
//		ms_Pos_Act.write_data({ (act_0) });
//		ms_Pos_Act2.write_data({ (act_1) });
//		//ms_Pos_Act.write_data({ (pos_act) });
//
//		amps_[0].set_current(amps_0);
//		amps_[1].set_current(amps_1);
//		amps_[2].set_current(amps_2);
//		amps_[3].set_current(amps_3);
//		amps_[4].set_current(amps_4);
//		amps_[5].set_current(amps_5);
//		amps_[6].set_current(amps_6);
//
//		//ms.write_data({ ref, act, act2, amps, amps2 });
//
//		//////////////////////////////////////////////////////////////////
//		// check for exit key
//		if (Keyboard::is_key_pressed(Key::Escape)) {
//			g_stop = true;
//		}
//
//		// update hardware
//		q8_.update_output();
//
//		// wait for remainder of sample period
//		timer.wait();
//
//	} // end control loop
//
//	  //////////////////////////////////
//}  //End start_myo function without the rest of the code below
//
//
///*
//while (!g_stop && !Keyboard::is_key_pressed(Key::Escape)) {
//
////MelShare Spar_pred1_label("Spar_pred2_label");
////Spar_pred1_label.write_data({ (double)((signed)(get_pred_label())) });
////{
////Lock lock(mtx);
////	std::cout << "But SparGlove thinks pred_label is" << std::endl;
//
//////Prove that we can see pred_label
//
////	std::cout << get_pred_label() << std::endl;
//
////	print("SparGlove Main loop");
////}
////std::cout << "MYO_Classifier thinks pred_label is" << std::endl;
//
////std::cout << pred_label << std::endl;
//
//
//// update hardware
//q8_.update_input();
//q8_.watchdog.kick();
//
//
//double torque = 1.0;
//
//switch (get_pred_label()) {
//case (size_t)0:
//torque = 0.0;
//break;
//case (size_t)1:
//torque = 0.1;
//break;
//case (size_t)2:
//torque = 0.2;
//break;
//case (size_t)3:
//torque = 0.3;
//break;
//case (size_t)4:
//torque = 0.4;
//break;
//case (size_t)5:
//torque = 0.5;
//break;
//case (size_t)6:
//torque = 0.6;
//break;
//
//default:
//torque = 0.9;
//
//}
//
////	std::array<double, 7> sat_torques = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; // temporary saturation torques
//double sat = 5.0;
//
//// experiment 1 code
//double pos_ref = get_joint(1).get_position();
//double pos_act = get_joint(0).get_position();
//double vel_act = get_joint(0).get_velocity();
//double torque2 = pd_controllers_[0].calculate(pos_ref, pos_act, 0, vel_act);
//
//double nothing = 0.0;
////		torque = saturate(torque, sat);
//torque2 = saturate(torque2, sat);
//
//get_joint(3).set_torque(torque);
//get_joint(0).set_torque(torque);
//
//to_ms_data[0] = pos_ref;
//to_ms_data[1] = pos_act;
//to_ms_data[2] = nothing;  // vel_act;
//to_ms_data[3] = torque2;
//
//ms.write_data(to_ms_data);
//
////ms.write_data({ get_joint(0).get_position() });
////ms.write_data({ get_joint(1).get_position() });
//
//// update output
//q8_.update_output();
//
//
//// wait timer
//timer.wait();
//}
//print("STARTED ENDED!!");
//};
//*/
//
//void SparGlove::start_turning() {
//	Time Ts = milliseconds(1); // sample period
//	std::size_t pred_label = 0;
//	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };
//
//	// construct clock to regulate interaction
//	Clock keypress_refract_clock;
//	Time keypress_refract_time = seconds(0.2);
//
//	// make MelShares
//	//MelShare ms_pred_label("pred_label");
//	//MelShare ms_Torque("Torque");
//	MelShare ms_Pos_Ref("Amps");
//	//MelShare ms_Pos_Act("Mot2");
//
//	// construct timer in hybrid mode to avoid using 100% CPU
//	Timer timer(Ts, Timer::Hybrid);
//	timer.set_acceptable_miss_rate(0.3);
//
//	q8_.encoder[0].zero();
//	q8_.encoder[1].zero();
//	q8_.encoder[2].zero();
//	q8_.encoder[3].zero();
//	q8_.encoder[4].zero();
//	q8_.encoder[5].zero();
//	q8_.encoder[6].zero();
//
//	q8_.watchdog.start();
//
//	double ref = 0.0;
//	double ref2 = 0.0;
//
//	int case0 = 0;
//	int case1 = 0;
//	int case2 = 0;
//	int case3 = 0;
//	int case4 = 0;
//	int case5 = 0;
//	int case6 = 0;
//
//	//Step-wise points to feed into the PD controllers
//	double setpoint_0 = 0.0;
//	double setpoint_1 = 0.0;
//	double setpoint_2 = 0.0;
//	double setpoint_3 = 0.0;
//	double setpoint_4 = 0.0;
//	double setpoint_5 = 0.0;
//	double setpoint_6 = 0.0;
//
//	//Goal positions for each pose
//	double goal_0 = 0.0;
//	double goal_1 = 0.0;
//	double goal_2 = 0.0;
//	double goal_3 = 0.0;
//	double goal_4 = 0.0;
//	double goal_5 = 0.0;
//	double goal_6 = 0.0;
//
//	while (!g_stop) {
//
//		// update hardware
//		q8_.update_input();
//		q8_.watchdog.kick();
//
//		if (Keyboard::is_key_pressed(Key::E)) {
//			motors_[0].enable();
//			motors_[1].enable();
//			motors_[2].enable();
//			motors_[3].enable();
//			motors_[4].enable();
//			motors_[5].enable();
//			motors_[6].enable();
//		}
//		else if (Keyboard::is_key_pressed(Key::D)) {
//			motors_[0].disable();
//			motors_[1].disable();
//			motors_[2].disable();
//			motors_[3].disable();
//			motors_[4].disable();
//			motors_[5].disable();
//			motors_[6].disable();
//		}
//
//		//////////////////////////////////////////////////////////
//
//		double act_0 = get_joint(0).get_position();
//		double act_1 = get_joint(1).get_position();
//		double act_2 = get_joint(2).get_position();
//		double act_3 = get_joint(3).get_position();
//		double act_4 = get_joint(4).get_position();
//		double act_5 = get_joint(5).get_position();
//		double act_6 = get_joint(6).get_position();
//
//		double vel_0 = get_joint(0).get_velocity();
//		double vel_1 = get_joint(1).get_velocity();
//		double vel_2 = get_joint(2).get_velocity();
//		double vel_3 = get_joint(3).get_velocity();
//		double vel_4 = get_joint(4).get_velocity();
//		double vel_5 = get_joint(5).get_velocity();
//		double vel_6 = get_joint(6).get_velocity();
//
//		if (Keyboard::is_key_pressed(Key::E)) {
//			motors_[0].enable();
//			motors_[1].enable();
//			motors_[2].enable();
//			motors_[3].enable();
//			motors_[4].enable();
//			motors_[5].enable();
//			motors_[6].enable();
//		}
//		else if (Keyboard::is_key_pressed(Key::D)) {
//			motors_[0].disable();
//			motors_[1].disable();
//			motors_[2].disable();
//			motors_[3].disable();
//			motors_[4].disable();
//			motors_[5].disable();
//			motors_[6].disable();
//		}
//
//		//fake prediction
//		for (std::size_t k = 0; k < active_keys.size(); ++k) {
//			if (Keyboard::is_key_pressed(active_keys[k])) {
//				if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
//					pred_label = k;
//					std::cout << pred_label << std::endl;
//					keypress_refract_clock.restart();
//				}
//			}
//		}
//		//Old Switch Case
//		/*
//		switch (pred_label) {
//		case (size_t)0: //Reposition (Extended Palm)
//			case0++;
//			//torque = 0.2;
//
//			//std::cout << "case0 = ";
//			//std::cout << case0 << std::endl;
//
//			if (case0 > 1000) {
//
//				setpoint_0 = 0; // Thumb Flexion
//				setpoint_1 = 0; // Index Flexion
//				setpoint_2 = 0; // Secondary Flexion
//				setpoint_3 = 0; // Thumb Opposition
//				setpoint_4 = 0; // Thumb Extension
//				setpoint_5 = 0; // Index Extension
//				setpoint_6 = 0; // Secondary Extension
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)1: //Hook (Thumbs Up)
//			ref += 0.01;
//			ref2 += 0.01;
//			//torque = 0.2;
//			case1++;
//
//			if (case1 > 1000) {
//
//				setpoint_0 = 0; // Thumb Flexion
//				setpoint_1 = 60; // Index Flexion
//				setpoint_2 = 60; // Secondary Flexion
//				setpoint_3 = 0; // Thumb Opposition
//				setpoint_4 = 0; // Thumb Extension
//				setpoint_5 = -60; // Index Extension
//				setpoint_6 = -60; // Secondary Extension
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			break;
//		case (size_t)2: //Lat Pinch (Button Press)
//			ref -= 0.01;
//			ref2 -= 0.01;
//			//torque = 0.2;
//			case2++;
//			//torque = 0.2;
//
//			if (case2 > 1000) {
//
//				setpoint_0 = 60; // Thumb Flexion
//				setpoint_1 = 60; // Index Flexion
//				setpoint_2 = 60; // Secondary Flexion
//				setpoint_3 = 0; // Thumb Opposition
//				setpoint_4 = -60; // Thumb Extension
//				setpoint_5 = -60; // Index Extension
//				setpoint_6 = -60; // Secondary Extension
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)3: //Point
//			ref += 0.01;
//			//torque = 0.3;
//			case3++;
//			//torque = 0.2;
//
//			if (case3 > 1000) {
//
//				setpoint_0 = 60; // Thumb Flexion
//				setpoint_1 = 0; // Index Flexion
//				setpoint_2 = 60; // Secondary Flexion
//				setpoint_3 = 60; // Thumb Opposition
//				setpoint_4 = -60; // Thumb Extension
//				setpoint_5 = 0; // Index Extension
//				setpoint_6 = -60; // Secondary Extension
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)4: //Thumb Opposition
//			ref2 += 0.01;
//			//torque = 0.4;
//			case4++;
//			//torque = 0.2;
//
//			if (case4 > 1000) {
//
//				setpoint_0 = 0; // Thumb Flexion
//				setpoint_1 = 0; // Index Flexion
//				setpoint_2 = 0; // Secondary Flexion
//				setpoint_3 = 60; // Thumb Opposition
//				setpoint_4 = -60; // Thumb Extension
//				setpoint_5 = 0; // Index Extension
//				setpoint_6 = 0; // Secondary Extension
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)5: //Pinch (OK)
//			ref -= 0.01;
//			//torque = 0.2;
//			case5++;
//			//torque = 0.2;
//
//			if (case5 > 1000) {
//
//				setpoint_0 = 0; // Thumb Flexion
//				setpoint_1 = 60; // Index Flexion
//				setpoint_2 = 0; // Secondary Flexion
//				setpoint_3 = 60; // Thumb Opposition
//				setpoint_4 = 0; // Thumb Extension
//				setpoint_5 = -60; // Index Extension
//				setpoint_6 = 0; // Secondary Extension
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//		case (size_t)6: //Cylinder
//			ref2 -= 0.01;
//			//torque = 0.6;
//			case6++;
//			//torque = 0.2;
//
//			if (case6 > 1000) {
//
//				setpoint_0 = 0; // Thumb Flexion
//				setpoint_1 = 60; // Index Flexion
//				setpoint_2 = 0; // Secondary Flexion
//				setpoint_3 = 60; // Thumb Opposition
//				setpoint_4 = 0; // Thumb Extension
//				setpoint_5 = -60; // Index Extension
//				setpoint_6 = 0; // Secondary Extension
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//			break;
//
//		default:
//			double torque = 0.9;
//
//		}
//		*/
//
//		switch (pred_label) {
//		case (size_t)0: //Reposition (Extended Palm)
//			case0++;
//			//torque = 0.2;
//
//			//std::cout << "case0 = ";
//			//std::cout << case0 << std::endl;
//
//			if (case0 > 1000) {
//
//				goal_0 = 0;
//				goal_1 = 0;
//				goal_2 = 0;
//				goal_3 = 0;
//				goal_4 = 0;
//				goal_5 = 0;
//				goal_6 = 0;
//
//				setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
//				setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
//				setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
//				setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
//				setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
//				setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
//				setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension
//
//			}
//			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
//				std::abs(goal_1 - setpoint_1) > 0.05 &&
//				std::abs(goal_2 - setpoint_2) > 0.05 &&
//				std::abs(goal_3 - setpoint_3) > 0.05 &&
//				std::abs(goal_4 - setpoint_4) > 0.05 &&
//				std::abs(goal_5 - setpoint_5) > 0.05 &&
//				std::abs(goal_6 - setpoint_6) > 0.05) {
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//
//			}
//
//			break;
//		case (size_t)1:  //Hook (Thumbs Up)
//			goal_0 = 0;
//			goal_1 = 60;
//			goal_2 = 60;
//			goal_3 = 0;
//			goal_4 = 0;
//			goal_5 = -60;
//			goal_6 = -60;
//
//			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
//			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
//			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
//			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
//			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
//			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
//			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension
//
//
//			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
//				std::abs(goal_1 - setpoint_1) > 0.05 &&
//				std::abs(goal_2 - setpoint_2) > 0.05 &&
//				std::abs(goal_3 - setpoint_3) > 0.05 &&
//				std::abs(goal_4 - setpoint_4) > 0.05 &&
//				std::abs(goal_5 - setpoint_5) > 0.05 &&
//				std::abs(goal_6 - setpoint_6) > 0.05) {
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//
//			}
//
//			break;
//		case (size_t)2: //Lat Pinch (Button Press)
//			goal_0 = 60;
//			goal_1 = 60;
//			goal_2 = 60;
//			goal_3 = 0;
//			goal_4 = -60;
//			goal_5 = -60;
//			goal_6 = -60;
//
//			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
//			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
//			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
//			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
//			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
//			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
//			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension
//
//
//			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
//				std::abs(goal_1 - setpoint_1) > 0.05 &&
//				std::abs(goal_2 - setpoint_2) > 0.05 &&
//				std::abs(goal_3 - setpoint_3) > 0.05 &&
//				std::abs(goal_4 - setpoint_4) > 0.05 &&
//				std::abs(goal_5 - setpoint_5) > 0.05 &&
//				std::abs(goal_6 - setpoint_6) > 0.05) {
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//
//			}
//			break;
//		case (size_t)3: //Point
//			goal_0 = 60;
//			goal_1 = 0;
//			goal_2 = 60;
//			goal_3 = 60;
//			goal_4 = -60;
//			goal_5 = 0;
//			goal_6 = -60;
//
//			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
//			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
//			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
//			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
//			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
//			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
//			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension
//
//
//			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
//				std::abs(goal_1 - setpoint_1) > 0.05 &&
//				std::abs(goal_2 - setpoint_2) > 0.05 &&
//				std::abs(goal_3 - setpoint_3) > 0.05 &&
//				std::abs(goal_4 - setpoint_4) > 0.05 &&
//				std::abs(goal_5 - setpoint_5) > 0.05 &&
//				std::abs(goal_6 - setpoint_6) > 0.05) {
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//
//			}
//			break;
//		case (size_t)4: //Thumb Opposition
//			goal_0 = 0;
//			goal_1 = 0;
//			goal_2 = 0;
//			goal_3 = 60;
//			goal_4 = -0;
//			goal_5 = 0;
//			goal_6 = 0;
//
//			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
//			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
//			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
//			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
//			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
//			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
//			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension
//
//
//			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
//				std::abs(goal_1 - setpoint_1) > 0.05 &&
//				std::abs(goal_2 - setpoint_2) > 0.05 &&
//				std::abs(goal_3 - setpoint_3) > 0.05 &&
//				std::abs(goal_4 - setpoint_4) > 0.05 &&
//				std::abs(goal_5 - setpoint_5) > 0.05 &&
//				std::abs(goal_6 - setpoint_6) > 0.05)
//			{
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//
//			}
//			break;
//		case (size_t)5: //Pinch (OK)
//			goal_0 = 0;
//			goal_1 = 60;
//			goal_2 = 0;
//			goal_3 = 60;
//			goal_4 = 0;
//			goal_5 = -60;
//			goal_6 = 0;
//
//			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
//			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
//			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
//			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
//			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
//			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
//			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension
//
//
//			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
//				std::abs(goal_1 - setpoint_1) > 0.05 &&
//				std::abs(goal_2 - setpoint_2) > 0.05 &&
//				std::abs(goal_3 - setpoint_3) > 0.05 &&
//				std::abs(goal_4 - setpoint_4) > 0.05 &&
//				std::abs(goal_5 - setpoint_5) > 0.05 &&
//				std::abs(goal_6 - setpoint_6) > 0.05) {
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//
//			}
//			break;
//		case (size_t)6: //Cylinder
//			goal_0 = 0;
//			goal_1 = 60;
//			goal_2 = 0;
//			goal_3 = 60;
//			goal_4 = 0;
//			goal_5 = -60;
//			goal_6 = 0;
//
//			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
//			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
//			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
//			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
//			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
//			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
//			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension
//
//
//			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
//				std::abs(goal_1 - setpoint_1) > 0.05 &&
//				std::abs(goal_2 - setpoint_2) > 0.05 &&
//				std::abs(goal_3 - setpoint_3) > 0.05 &&
//				std::abs(goal_4 - setpoint_4) > 0.05 &&
//				std::abs(goal_5 - setpoint_5) > 0.05 &&
//				std::abs(goal_6 - setpoint_6) > 0.05) {
//
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//			}
//
//			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
//				case0 = 0;
//				case1 = 0;
//				case2 = 0;
//				case3 = 0;
//				case4 = 0;
//				case5 = 0;
//				case6 = 0;
//
//			}
//			break;
//		case (size_t)7: //Wave.  Press W to activate
//
//
//
//			break;
//
//		default:
//			double torque = 0.9;
//
//		}
//
//
//		double amps_0 = pd_controllers_[0].calculate(setpoint_0, act_0, 0, vel_0);
//		double amps_1 = pd_controllers_[1].calculate(setpoint_1, act_1, 0, vel_1);
//		double amps_2 = pd_controllers_[2].calculate(setpoint_2, act_2, 0, vel_2);
//		double amps_3 = pd_controllers_[3].calculate(setpoint_3, act_3, 0, vel_3);
//		double amps_4 = pd_controllers_[4].calculate(setpoint_4, act_4, 0, vel_4);
//		double amps_5 = pd_controllers_[5].calculate(setpoint_5, act_5, 0, vel_5);
//		double amps_6 = pd_controllers_[6].calculate(setpoint_6, act_6, 0, vel_6);
//
//		double sat = 0.25;
//		amps_0 = -saturate(amps_0, -0.17, 0.17);
//		amps_1 = -saturate(amps_1, -0.17, 0.17);
//		amps_2 = -saturate(amps_2, -0.17, 0.17);
//		amps_3 = -saturate(amps_3, -0.17, 0.17);
//		amps_4 = -saturate(amps_4, -0.17, 0.17);
//		amps_5 = -saturate(amps_5, -0.17, 0.17);
//		amps_6 = -saturate(amps_6, -0.17, 0.17);
//
//		if (std::abs(amps_0) > 0.18) {
//			g_stop = true;
//		}
//
//		// write to MelShares
//		//ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
//		//ms_Ref.write_data({ (ref) });
//		//ms_Ref2.write_data({ (ref2) });
//		//ms_Amps.write_data({ (amps_0) });
//		//ms_Pos_Act.write_data({ (act_0) });
//		//ms_Pos_Act2.write_data({ (act_1) });
//		//ms_Pos_Act.write_data({ (pos_act) });
//
//		/*amps_[0].set_current(amps_0);
//		amps_[1].set_current(amps_1);
//		amps_[2].set_current(amps_2);
//		amps_[3].set_current(amps_3);
//		amps_[4].set_current(amps_4);
//		amps_[5].set_current(amps_5);
//		amps_[6].set_current(amps_6);*/
//
//
//
//		//////////////////////////////////////////////////////////
//
//
//
//				/*if (Keyboard::is_key_pressed(Key::Up)) {
//					ref += 0.01;
//					ref2 += 0.01;
//				}
//				else if (Keyboard::is_key_pressed(Key::Down)) {
//					ref -= 0.01;
//					ref2 -= 0.01;
//				}*/
//
//		double pos_ref = get_joint(1).get_position();
//		double pos_act = get_joint(0).get_position();
//		double vel_act = get_joint(0).get_velocity();
//		double torque = pred_label * 0.1;
//		double amps = pd_controllers_[0].calculate(ref, pos_act, 0, vel_act);
//
//		//amps = saturate(amps, sat);
//		//amps = -saturate(amps, -0.17, 0.17);
//		amps_[0].set_current(amps_0);
//		amps_[1].set_current(amps_1);
//		amps_[2].set_current(amps_2);
//		amps_[3].set_current(amps_3);
//		amps_[4].set_current(amps_4);
//		amps_[5].set_current(amps_5);
//		amps_[6].set_current(amps_6);
//
//		// write to MelShares
//		//ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
//		//ms_Torque.write_data({ (torque) });
//		ms_Pos_Ref.write_data({ amps_0, amps_1, amps_2, amps_3, amps_4, amps_5, amps_6 });
//		//ms_Pos_Act.write_data({ (pos_act) });
//
//		//if (Keyboard::is_key_pressed(Key::Z)) {
//		//	q8_.encoder[0].zero();
//		//	q8_.encoder[1].zero(); 
//		//							
//		//	std::cout << "Encoders are zeroed" << std::endl;
//		//}
//
//		// check for exit key
//		if (Keyboard::is_key_pressed(Key::Escape)) {
//			g_stop = true;
//		}
//
//		// update hardware
//		q8_.update_output();
//
//		// wait for remainder of sample period
//		timer.wait();
//
//	}
//
//
//}
//
void SparGlove::instrumented_hand() {

	Q8Usb q8_0;
	Q8Usb* q8_A;
	Q8Usb* q8_B;

	Time Ts = milliseconds(1); // sample period
	std::size_t pred_label = 0;
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };

	// construct clock to regulate interaction
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.2);

	// make MelShares

	MelShare ms_Instrumented("Hand");

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);
	timer.set_acceptable_miss_rate(0.3);

	// open Q8s

	q8_0.open();
	q8_.open();

	if (q8_0.identify(0))
		q8_A = &q8_0;
	else if (q8_0.identify(7))
		q8_B = &q8_0;

	if (q8_.identify(0))
		q8_A = &q8_;
	else if (q8_.identify(7))
		q8_B = &q8_;

	q8_A->enable();
	q8_B->enable();

	std::array<std::array<double, 18>, 1> data;



	q8_.encoder[0].zero();
	q8_.encoder[1].zero();
	q8_.encoder[2].zero();
	q8_.encoder[3].zero();
	q8_.encoder[4].zero();
	q8_.encoder[5].zero();
	q8_.encoder[6].zero();

	q8_.watchdog.start();
	q8_0.watchdog.start();
	double ref = 0.0;
	double ref2 = 0.0;

	int case0 = 0;
	int case1 = 0;
	int case2 = 0;
	int case3 = 0;
	int case4 = 0;
	int case5 = 0;
	int case6 = 0;

	double setpoint_0 = 0.0;
	double setpoint_1 = 0.0;
	double setpoint_2 = 0.0;
	double setpoint_3 = 0.0;
	double setpoint_4 = 0.0;
	double setpoint_5 = 0.0;
	double setpoint_6 = 0.0;

	double goal_0 = 0.0;
	double goal_1 = 0.0;
	double goal_2 = 0.0;
	double goal_3 = 0.0;
	double goal_4 = 0.0;
	double goal_5 = 0.0;
	double goal_6 = 0.0;

	Waveform sin_wave(Waveform::Sin, seconds(2));


	// container for header
	std::vector<std::string> header;
	header.push_back("Thumb 1");
	header.push_back("Thumb 2");
	header.push_back("Thumb 3");
	header.push_back("Thumb 4");
	header.push_back("Index 1");
	header.push_back("Index 2");
	header.push_back("Middle 1");
	header.push_back("Middle 2");

	header.push_back("Middle Str Pot");
	header.push_back("Index Str Pot");
	header.push_back("Thumb Str Pot");

	header.push_back("Thumb Flexion (mm)");
	header.push_back("Index Flexion (mm)");
	header.push_back("Secondary Flexion (mm)");
	header.push_back("Thumb Opposition (mm)");
	header.push_back("Thumb Extension (mm)");
	header.push_back("Index Extension (mm)");
	header.push_back("Secondary Extension (mm)");

	// path to csv file (my_files dir will be created)
	std::string filepath = "my_files/saad.csv";
	// write the header
	csv_write_row(filepath, header);
	// append the data
	//csv_append_rows(filepath, data);



	while (!g_stop) {
		q8_A->update_input();
		q8_B->update_input();
		//mel::print(q2.analog_input.get_value(0));
		//mel::print(q2.analog_input.get_value(1));
		//double voltage = wave.evaluate(timer.get_elapsed_time());
		//q8.analog_output.set_value(0, 5.0);
		q8_A->AO.set_value(0, 5.0);

		//q8.analog_output.set_value(1, 5.0);
		q8_A->AO.set_value(1, 5.0);
		q8_B->AO.set_value(7, 5.0);

		//////////////////////////////////////////////////////////

		q8_A->watchdog.kick();
		q8_B->watchdog.kick();

		if (Keyboard::is_key_pressed(Key::E)) {
			motors_[0].enable();
			/*motors_[1].enable();
			motors_[2].enable();
			motors_[3].enable();
			motors_[4].enable();
			motors_[5].enable();
			motors_[6].enable();*/
		}
		else if (Keyboard::is_key_pressed(Key::D)) {
			motors_[0].disable();
			motors_[1].disable();
			motors_[2].disable();
			motors_[3].disable();
			motors_[4].disable();
			motors_[5].disable();
			motors_[6].disable();
		}

		double act_0 = get_joint(0).get_position();
		double act_1 = get_joint(1).get_position();
		double act_2 = get_joint(2).get_position();
		double act_3 = get_joint(3).get_position();
		double act_4 = get_joint(4).get_position();
		double act_5 = get_joint(5).get_position();
		double act_6 = get_joint(6).get_position();

		double vel_0 = get_joint(0).get_velocity();
		double vel_1 = get_joint(1).get_velocity();
		double vel_2 = get_joint(2).get_velocity();
		double vel_3 = get_joint(3).get_velocity();
		double vel_4 = get_joint(4).get_velocity();
		double vel_5 = get_joint(5).get_velocity();
		double vel_6 = get_joint(6).get_velocity();

		if (Keyboard::is_key_pressed(Key::E)) {
			motors_[0].enable();
			//motors_[1].enable();
			//motors_[2].enable();
			//motors_[3].enable();
			//motors_[4].enable();
			//motors_[5].enable();
			//motors_[6].enable();
		}
		else if (Keyboard::is_key_pressed(Key::D)) {
			motors_[0].disable();
			motors_[1].disable();
			motors_[2].disable();
			motors_[3].disable();
			motors_[4].disable();
			motors_[5].disable();
			motors_[6].disable();
		}

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

		if (Keyboard::is_key_pressed(Key::W)) {
			pred_label = 7;
		}

		switch (pred_label) {
		case (size_t)0: //Reposition (Extended Palm)
			case0++;
			//torque = 0.2;

			//std::cout << "case0 = ";
			//std::cout << case0 << std::endl;

			if (case0 > 1000) {

				goal_0 = 0;
				goal_1 = 0;
				goal_2 = 0;
				goal_3 = 0;
				goal_4 = 0;
				goal_5 = 0;
				goal_6 = 0;

				setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.05; // Thumb Flexion
				setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.05;; // Index Flexion
				setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.05;; // Secondary Flexion
				setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.05;; // Thumb Opposition
				setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.05;; // Thumb Extension
				setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.05;; // Index Extension
				setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.05;; // Secondary Extension

			}
			if (std::abs(goal_0 - setpoint_0) > 0.5 &&
				std::abs(goal_1 - setpoint_1) > 0.5 &&
				std::abs(goal_2 - setpoint_2) > 0.5 &&
				std::abs(goal_3 - setpoint_3) > 0.5 &&
				std::abs(goal_4 - setpoint_4) > 0.5 &&
				std::abs(goal_5 - setpoint_5) > 0.5 &&
				std::abs(goal_6 - setpoint_6) > 0.5) {

				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;
			}

			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;

			}

			break;
		case (size_t)1:  //Hook (Thumbs Up)
			goal_0 = 0;
			goal_1 = 60;
			goal_2 = 60;
			goal_3 = 0;
			goal_4 = 0;
			goal_5 = -60;
			goal_6 = -60;

			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension


			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
				std::abs(goal_1 - setpoint_1) > 0.05 &&
				std::abs(goal_2 - setpoint_2) > 0.05 &&
				std::abs(goal_3 - setpoint_3) > 0.05 &&
				std::abs(goal_4 - setpoint_4) > 0.05 &&
				std::abs(goal_5 - setpoint_5) > 0.05 &&
				std::abs(goal_6 - setpoint_6) > 0.05) {

				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;
			}

			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;

			}

			break;
		case (size_t)2: //Lat Pinch (Button Press)
			goal_0 = 20;
			goal_1 = 60;
			goal_2 = 60;
			goal_3 = 0;
			goal_4 = -60;
			goal_5 = -60;
			goal_6 = -60;

			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.05; // Thumb Flexion
			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.05;; // Index Flexion
			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.05;; // Secondary Flexion
			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.05;; // Thumb Opposition
			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.05;; // Thumb Extension
			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.05;; // Index Extension
			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.05;; // Secondary Extension


			if (std::abs(goal_0 - setpoint_0) > 0.5 &&
				std::abs(goal_1 - setpoint_1) > 0.5 &&
				std::abs(goal_2 - setpoint_2) > 0.5 &&
				std::abs(goal_3 - setpoint_3) > 0.5 &&
				std::abs(goal_4 - setpoint_4) > 0.5 &&
				std::abs(goal_5 - setpoint_5) > 0.5 &&
				std::abs(goal_6 - setpoint_6) > 0.5) {

				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;
			}

			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;

			}
			break;
		case (size_t)3: //Point
			goal_0 = 60;
			goal_1 = 0;
			goal_2 = 60;
			goal_3 = 60;
			goal_4 = -60;
			goal_5 = 0;
			goal_6 = -60;

			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension


			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
				std::abs(goal_1 - setpoint_1) > 0.05 &&
				std::abs(goal_2 - setpoint_2) > 0.05 &&
				std::abs(goal_3 - setpoint_3) > 0.05 &&
				std::abs(goal_4 - setpoint_4) > 0.05 &&
				std::abs(goal_5 - setpoint_5) > 0.05 &&
				std::abs(goal_6 - setpoint_6) > 0.05) {

				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;
			}

			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;

			}
			break;
		case (size_t)4: //Thumb Opposition
			goal_0 = 0;
			goal_1 = 0;
			goal_2 = 0;
			goal_3 = 60;
			goal_4 = -0;
			goal_5 = 0;
			goal_6 = 0;

			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension


			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
				std::abs(goal_1 - setpoint_1) > 0.05 &&
				std::abs(goal_2 - setpoint_2) > 0.05 &&
				std::abs(goal_3 - setpoint_3) > 0.05 &&
				std::abs(goal_4 - setpoint_4) > 0.05 &&
				std::abs(goal_5 - setpoint_5) > 0.05 &&
				std::abs(goal_6 - setpoint_6) > 0.05)
			{
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;
			}

			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;

			}
			break;
		case (size_t)5: //Pinch (OK)
			goal_0 = 0;
			goal_1 = 60;
			goal_2 = 0;
			goal_3 = 60;
			goal_4 = 0;
			goal_5 = -60;
			goal_6 = 0;

			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension


			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
				std::abs(goal_1 - setpoint_1) > 0.05 &&
				std::abs(goal_2 - setpoint_2) > 0.05 &&
				std::abs(goal_3 - setpoint_3) > 0.05 &&
				std::abs(goal_4 - setpoint_4) > 0.05 &&
				std::abs(goal_5 - setpoint_5) > 0.05 &&
				std::abs(goal_6 - setpoint_6) > 0.05) {

				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;
			}

			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;

			}
			break;
		case (size_t)6: //Cylinder
			goal_0 = 0;
			goal_1 = 60;
			goal_2 = 0;
			goal_3 = 60;
			goal_4 = 0;
			goal_5 = -60;
			goal_6 = 0;

			setpoint_0 = setpoint_0 - sign(act_0 - goal_0)*0.01; // Thumb Flexion
			setpoint_1 = setpoint_1 - sign(act_1 - goal_1)*0.01;; // Index Flexion
			setpoint_2 = setpoint_2 - sign(act_2 - goal_2)*0.01;; // Secondary Flexion
			setpoint_3 = setpoint_3 - sign(act_3 - goal_3)*0.01;; // Thumb Opposition
			setpoint_4 = setpoint_4 - sign(act_4 - goal_4)*0.01;; // Thumb Extension
			setpoint_5 = setpoint_5 - sign(act_5 - goal_5)*0.01;; // Index Extension
			setpoint_6 = setpoint_6 - sign(act_6 - goal_6)*0.01;; // Secondary Extension


			if (std::abs(goal_0 - setpoint_0) > 0.05 &&
				std::abs(goal_1 - setpoint_1) > 0.05 &&
				std::abs(goal_2 - setpoint_2) > 0.05 &&
				std::abs(goal_3 - setpoint_3) > 0.05 &&
				std::abs(goal_4 - setpoint_4) > 0.05 &&
				std::abs(goal_5 - setpoint_5) > 0.05 &&
				std::abs(goal_6 - setpoint_6) > 0.05) {

				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;
			}

			if (Keyboard::is_key_pressed(Key::Space)) { //Hit SpaceBar to abandon the pose
				case0 = 0;
				case1 = 0;
				case2 = 0;
				case3 = 0;
				case4 = 0;
				case5 = 0;
				case6 = 0;

			}
			break;
		case (size_t)7: //Wave.  Press W to activate



			break;

		default:
			setpoint_0 = act_0;
			setpoint_1 = act_1;
			setpoint_2 = act_2;
			setpoint_3 = act_3;
			setpoint_4 = act_4;
			setpoint_5 = act_5;
			setpoint_6 = act_6;
		}

		double amps_0 = pd_controllers_[0].calculate(setpoint_0, act_0, 0, vel_0);
		double amps_1 = pd_controllers_[1].calculate(setpoint_1, act_1, 0, vel_1);
		double amps_2 = pd_controllers_[2].calculate(setpoint_2, act_2, 0, vel_2);
		double amps_3 = pd_controllers_[3].calculate(setpoint_3, act_3, 0, vel_3);
		double amps_4 = pd_controllers_[4].calculate(setpoint_4, act_4, 0, vel_4);
		double amps_5 = pd_controllers_[5].calculate(setpoint_5, act_5, 0, vel_5);
		double amps_6 = pd_controllers_[6].calculate(setpoint_6, act_6, 0, vel_6);

		double sat = 0.25;
		amps_0 = -saturate(amps_0, -0.17, 0.17);
		amps_1 = -saturate(amps_1, -0.17, 0.17);
		amps_2 = -saturate(amps_2, -0.17, 0.17);
		amps_3 = -saturate(amps_3, -0.17, 0.17);
		amps_4 = -saturate(amps_4, -0.17, 0.17);
		amps_5 = -saturate(amps_5, -0.17, 0.17);
		amps_6 = -saturate(amps_6, -0.17, 0.17);

		if (std::abs(amps_0) > 0.18) {
			g_stop = true;
		}


		/*if (Keyboard::is_key_pressed(Key::Up)) {
		ref += 0.01;
		ref2 += 0.01;
		}
		else if (Keyboard::is_key_pressed(Key::Down)) {
		ref -= 0.01;
		ref2 -= 0.01;
		}*/

		/*	double pos_ref = get_joint(1).get_position();
			double pos_act = get_joint(0).get_position();
			double vel_act = get_joint(0).get_velocity();
			double torque = pred_label * 0.1;
			double amps = pd_controllers_[0].calculate(ref, pos_act, 0, vel_act);*/

			//amps = saturate(amps, sat);
			//amps = -saturate(amps, -0.17, 0.17);
		amps_[0].set_current(amps_0);
		amps_[1].set_current(amps_1);
		amps_[2].set_current(amps_2);
		amps_[3].set_current(amps_3);
		amps_[4].set_current(amps_4);
		amps_[5].set_current(amps_5);
		amps_[6].set_current(amps_6);

		// write to MelShares
		//ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
		//ms_Torque.write_data({ (torque) });
		ms_Instrumented.write_data({ amps_0, amps_1, amps_2, amps_3, amps_4, amps_5, amps_6 });
		//ms_Pos_Act.write_data({ (pos_act) });

		//if (Keyboard::is_key_pressed(Key::Z)) {
		//	q8_.encoder[0].zero();
		//	q8_.encoder[1].zero(); 
		//							
		//	std::cout << "Encoders are zeroed" << std::endl;
		//}

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			g_stop = true;
		}



		//////////////////////////////////////////////////////////
		q8_A->update_output();
		q8_B->update_output();
		timer.wait();

		data[0][0] = q8_A->AI.get_value(0);
		data[0][1] = q8_A->AI.get_value(1);
		data[0][2] = q8_A->AI.get_value(2);
		data[0][3] = q8_A->AI.get_value(3);
		data[0][4] = q8_A->AI.get_value(4);
		data[0][5] = q8_A->AI.get_value(5);
		data[0][6] = q8_A->AI.get_value(6);
		data[0][7] = q8_A->AI.get_value(7);
		// for (std::size_t c = 0; c < 8; ++c) {
		//     dataarray[c] = data[c];
		// }

		data[0][8] = q8_B->AI.get_value(0);
		data[0][9] = q8_B->AI.get_value(1);
		data[0][10] = q8_B->AI.get_value(2);

		data[0][11] = get_joint(0).get_position(); //Motor position in millimeters
		data[0][12] = get_joint(1).get_position();
		data[0][13] = get_joint(2).get_position();
		data[0][14] = get_joint(3).get_position();
		data[0][15] = get_joint(4).get_position();
		data[0][16] = get_joint(5).get_position();
		data[0][17] = get_joint(6).get_position();

		csv_append_rows(filepath, data);
	}
}
/*
void SparGlove::start_homing() {
	Time Ts = milliseconds(1); // sample period
	std::size_t pred_label = 0;
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };

	// construct clock to regulate interaction
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.2);

	// make MelShares
	//MelShare ms_pred_label("pred_label");
	//MelShare ms_Torque("Torque");
	MelShare ms_Pos_Ref("Amps");
	//MelShare ms_Pos_Act("Pos_Act");

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);

	Time t;

	q8_.watchdog.start();

	double pos_ref = 0;
	double setpoint_0 = 0.0;
	double setpoint_1 = 0.0;
	double setpoint_2 = 0.0;
	double setpoint_3 = 0.0;
	double setpoint_4 = 0.0;
	double setpoint_5 = 0.0;
	double setpoint_6 = 0.0;

	q8_.encoder[0].zero();
	q8_.encoder[1].zero();
	q8_.encoder[2].zero();
	q8_.encoder[3].zero();
	q8_.encoder[4].zero();
	q8_.encoder[5].zero();
	q8_.encoder[6].zero();

	bool home = false;
	int count = 0;

	//////////////////// HOMING LOOP //////////////////////////
	while (!g_stop) {

		t = timer.get_elapsed_time();

		if (home == true) {
			count++;
		}

		// update hardware
		q8_.watchdog.kick();
		q8_.update_input();

		if (Keyboard::is_key_pressed(Key::E)) {
			motors_[0].enable();
			motors_[1].enable();
			motors_[2].enable();
			motors_[3].enable();
			motors_[4].enable();
			motors_[5].enable();
			motors_[6].enable();

		}
		else if (Keyboard::is_key_pressed(Key::D)) {
			motors_[0].disable();
			motors_[1].disable();
			motors_[2].disable();
			motors_[3].disable();
			motors_[4].disable();
			motors_[5].disable();
			motors_[6].disable();
		}
		//double pos_act = get_joint(0).get_position();
		//double vel_act = get_joint(0).get_velocity();

		double act_0 = get_joint(0).get_position();
		double act_1 = get_joint(1).get_position();
		double act_2 = get_joint(2).get_position();
		double act_3 = get_joint(3).get_position();
		double act_4 = get_joint(4).get_position();
		double act_5 = get_joint(5).get_position();
		double act_6 = get_joint(6).get_position();

		double vel_0 = get_joint(0).get_velocity();
		double vel_1 = get_joint(1).get_velocity();
		double vel_2 = get_joint(2).get_velocity();
		double vel_3 = get_joint(3).get_velocity();
		double vel_4 = get_joint(4).get_velocity();
		double vel_5 = get_joint(5).get_velocity();
		double vel_6 = get_joint(6).get_velocity();

		if (Keyboard::is_key_pressed(Key::H)) {
			setpoint_0--; // Thumb Flexion
			setpoint_1--; // Index Flexion
			setpoint_2--; // Secondary Flexion
			setpoint_3--; // Thumb Opposition
			setpoint_4++; // Thumb Extension
			setpoint_5++; // Index Extension
			setpoint_6++; // Secondary Extension

			//setpoint_0 = -60; // Thumb Flexion
			//setpoint_1 = -60; // Index Flexion
			//setpoint_2 = -60; // Secondary Flexion
			//setpoint_3 = -60; // Thumb Opposition
			//setpoint_4 = 60; // Thumb Extension
			//setpoint_5 = 60; // Index Extension
			//setpoint_6 = 60; // Secondary Extension
			home = true;

			std::cout << "H is pressed" << std::endl;
		}
		//else if (Keyboard::is_key_pressed(Key::Z)) {
		//	pos_ref = 0;
		//	std::cout << "Z is pressed" << std::endl;
		//}
		//else {
		//	//pos_ref = pos_act;
		//}

		double amps_0 = pd_controllers_[0].calculate(setpoint_0, act_0, 0, vel_0);
		double amps_1 = pd_controllers_[1].calculate(setpoint_1, act_1, 0, vel_1);
		double amps_2 = pd_controllers_[2].calculate(setpoint_2, act_2, 0, vel_2);
		double amps_3 = pd_controllers_[3].calculate(setpoint_3, act_3, 0, vel_3);
		double amps_4 = pd_controllers_[4].calculate(setpoint_4, act_4, 0, vel_4);
		double amps_5 = pd_controllers_[5].calculate(setpoint_5, act_5, 0, vel_5);
		double amps_6 = pd_controllers_[6].calculate(setpoint_6, act_6, 0, vel_6);

		//double count = t.as_milliseconds();
		double standin_count = count;
		ms_Pos_Ref.write_data({ standin_count, vel_4, amps_2, amps_3, amps_4, get_joint(4).get_position(), get_joint(4).get_velocity() });

		double shoulder = 1.0;

		if (count >= 5000 &&
			std::abs(vel_0) <= shoulder &&
			std::abs(vel_1) <= shoulder &&
			std::abs(vel_2) <= shoulder &&
			std::abs(vel_3) <= shoulder &&
			std::abs(vel_4) <= shoulder &&
			std::abs(vel_5) <= shoulder &&
			std::abs(vel_6) <= shoulder) {

			std::cout << "Torque Limits reached" << std::endl;
			break;
		}

		double sat = .1;

		amps_0 = -saturate(amps_0, -sat, sat);
		amps_1 = -saturate(amps_1, -sat, sat);
		amps_2 = -saturate(amps_2, -sat, sat);
		amps_3 = -saturate(amps_3, -sat, sat);
		amps_4 = -saturate(amps_4, -sat, sat);
		amps_5 = -saturate(amps_5, -sat, sat);
		amps_6 = -saturate(amps_6, -sat, sat);


		//torque2 = saturate(torque2, sat);

		amps_[0].set_current(amps_0);
		amps_[1].set_current(amps_1);
		amps_[2].set_current(amps_2);
		amps_[3].set_current(amps_3);
		amps_[4].set_current(amps_4);
		amps_[5].set_current(amps_5);
		amps_[6].set_current(amps_6);

		//get_joint(1).set_torque(torque2);
		// write to MelShares
		//ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });
		//ms_Torque.write_data({ (torque2) });
		//ms_Pos_Act.write_data({ (pos_act) });


		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			g_stop = true;
		}

		// update hardware
		q8_.update_output();

		// wait for remainder of sample period
		timer.wait();

	} //END WHILE LOOP

	//g_stop = false;

	motors_[0].enable();
	motors_[1].enable();
	motors_[2].enable();
	motors_[3].enable();
	//	motors_[4].enable();
	//	motors_[5].enable();
	//	motors_[6].enable();


	for (int i = 0; i <= 150; i++) {

		q8_.update_input();

		double act_0 = get_joint(0).get_position();
		double act_1 = get_joint(1).get_position();
		double act_2 = get_joint(2).get_position();
		double act_3 = get_joint(3).get_position();
		double act_4 = get_joint(4).get_position();
		double act_5 = get_joint(5).get_position();
		double act_6 = get_joint(6).get_position();

		double vel_0 = get_joint(0).get_velocity();
		double vel_1 = get_joint(1).get_velocity();
		double vel_2 = get_joint(2).get_velocity();
		double vel_3 = get_joint(3).get_velocity();
		double vel_4 = get_joint(4).get_velocity();
		double vel_5 = get_joint(5).get_velocity();
		double vel_6 = get_joint(6).get_velocity();

		setpoint_0++; // Thumb Flexion
		setpoint_1++; // Index Flexion
		setpoint_2++; // Secondary Flexion
		setpoint_3++; // Thumb Opposition
		//setpoint_4++; // Thumb Extension
		//setpoint_5++; // Index Extension
		//setpoint_6++; // Secondary Extension

		double amps_0 = pd_controllers_[0].calculate(setpoint_0, act_0, 0, vel_0);
		double amps_1 = pd_controllers_[1].calculate(setpoint_1, act_1, 0, vel_1);
		double amps_2 = pd_controllers_[2].calculate(setpoint_2, act_2, 0, vel_2);
		double amps_3 = pd_controllers_[3].calculate(setpoint_3, act_3, 0, vel_3);
		double amps_4 = pd_controllers_[4].calculate(setpoint_4, act_4, 0, vel_4);
		double amps_5 = pd_controllers_[5].calculate(setpoint_5, act_5, 0, vel_5);
		double amps_6 = pd_controllers_[6].calculate(setpoint_6, act_6, 0, vel_6);

		////double count = t.as_milliseconds();
		//double standin_count = count;
		//ms_Pos_Ref.write_data({ standin_count, vel_4, amps_2, amps_3, amps_4, get_joint(4).get_position(), get_joint(4).get_velocity() });

		//double shoulder = 1.0;

		//if (count >= 5000 &&
		//	std::abs(vel_0) <= shoulder &&
		//	std::abs(vel_1) <= shoulder &&
		//	std::abs(vel_2) <= shoulder &&
		//	std::abs(vel_3) <= shoulder &&
		//	std::abs(vel_4) <= shoulder &&
		//	std::abs(vel_5) <= shoulder &&
		//	std::abs(vel_6) <= shoulder) {

		//	std::cout << "Torque Limits reached" << std::endl;
		//	break;
		//}

		double sat = .1;

		amps_0 = -saturate(amps_0, -sat, sat);
		amps_1 = -saturate(amps_1, -sat, sat);
		amps_2 = -saturate(amps_2, -sat, sat);
		amps_3 = -saturate(amps_3, -sat, sat);
		amps_4 = -saturate(amps_4, -sat, sat);
		amps_5 = -saturate(amps_5, -sat, sat);
		amps_6 = -saturate(amps_6, -sat, sat);


		//torque2 = saturate(torque2, sat);

		amps_[0].set_current(amps_0);
		amps_[1].set_current(amps_1);
		amps_[2].set_current(amps_2);
		amps_[3].set_current(amps_3);
		amps_[4].set_current(amps_4);
		amps_[5].set_current(amps_5);
		amps_[6].set_current(amps_6);

		// update hardware
		q8_.update_output();

		// wait for remainder of sample period
		timer.wait();

		std::cout << "Made a loop through the FOR" << std::endl;


	} //END FOR LOOP





	std::cout << "Broke out of the FOR" << std::endl;
	// check for exit key
	if (Keyboard::is_key_pressed(Key::Escape)) {
		g_stop = true;
	}

}

void SparGlove::offset() {
	Time Ts = milliseconds(1); // sample period
	std::size_t pred_label = 0;
	std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7 };

	// construct clock to regulate interaction
	Clock keypress_refract_clock;
	Time keypress_refract_time = seconds(0.2);

	// make MelShares
	//MelShare ms_pred_label("pred_label");
	//MelShare ms_Torque("Torque");
	MelShare ms_Pos_Ref("Amps");
	//MelShare ms_Pos_Act("Pos_Act");

	// construct timer in hybrid mode to avoid using 100% CPU
	Timer timer(Ts, Timer::Hybrid);

	Time t;

	q8_.watchdog.start();

	motors_[0].enable();
	motors_[1].enable();
	motors_[2].enable();
	motors_[3].enable();
	//	motors_[4].enable();
	//	motors_[5].enable();
	//	motors_[6].enable();

	double pos_ref = 0;
	double setpoint_0 = 0.0;
	double setpoint_1 = 0.0;
	double setpoint_2 = 0.0;
	double setpoint_3 = 0.0;
	double setpoint_4 = 0.0;
	double setpoint_5 = 0.0;
	double setpoint_6 = 0.0;

	q8_.encoder[0].zero();
	q8_.encoder[1].zero();
	q8_.encoder[2].zero();
	q8_.encoder[3].zero();
	q8_.encoder[4].zero();
	q8_.encoder[5].zero();
	q8_.encoder[6].zero();

	bool home = false;
	int count = 0;

	for (int i = 0; i <= 15000; i++) {

		q8_.update_input();

		double act_0 = get_joint(0).get_position();
		double act_1 = get_joint(1).get_position();
		double act_2 = get_joint(2).get_position();
		double act_3 = get_joint(3).get_position();
		double act_4 = get_joint(4).get_position();
		double act_5 = get_joint(5).get_position();
		double act_6 = get_joint(6).get_position();

		double vel_0 = get_joint(0).get_velocity();
		double vel_1 = get_joint(1).get_velocity();
		double vel_2 = get_joint(2).get_velocity();
		double vel_3 = get_joint(3).get_velocity();
		double vel_4 = get_joint(4).get_velocity();
		double vel_5 = get_joint(5).get_velocity();
		double vel_6 = get_joint(6).get_velocity();

		setpoint_0++; // Thumb Flexion
		setpoint_1++; // Index Flexion
		setpoint_2++; // Secondary Flexion
		setpoint_3++; // Thumb Opposition
					  //setpoint_4++; // Thumb Extension
					  //setpoint_5++; // Index Extension
					  //setpoint_6++; // Secondary Extension

		double amps_0 = pd_controllers_[0].calculate(setpoint_0, act_0, 0, vel_0);
		double amps_1 = pd_controllers_[1].calculate(setpoint_1, act_1, 0, vel_1);
		double amps_2 = pd_controllers_[2].calculate(setpoint_2, act_2, 0, vel_2);
		double amps_3 = pd_controllers_[3].calculate(setpoint_3, act_3, 0, vel_3);
		double amps_4 = pd_controllers_[4].calculate(setpoint_4, act_4, 0, vel_4);
		double amps_5 = pd_controllers_[5].calculate(setpoint_5, act_5, 0, vel_5);
		double amps_6 = pd_controllers_[6].calculate(setpoint_6, act_6, 0, vel_6);

		////double count = t.as_milliseconds();
		//double standin_count = count;
		//ms_Pos_Ref.write_data({ standin_count, vel_4, amps_2, amps_3, amps_4, get_joint(4).get_position(), get_joint(4).get_velocity() });

		//double shoulder = 1.0;

		//if (count >= 5000 &&
		//	std::abs(vel_0) <= shoulder &&
		//	std::abs(vel_1) <= shoulder &&
		//	std::abs(vel_2) <= shoulder &&
		//	std::abs(vel_3) <= shoulder &&
		//	std::abs(vel_4) <= shoulder &&
		//	std::abs(vel_5) <= shoulder &&
		//	std::abs(vel_6) <= shoulder) {

		//	std::cout << "Torque Limits reached" << std::endl;
		//	break;
		//}

		double sat = .1;

		amps_0 = -saturate(amps_0, -sat, sat);
		amps_1 = -saturate(amps_1, -sat, sat);
		amps_2 = -saturate(amps_2, -sat, sat);
		amps_3 = -saturate(amps_3, -sat, sat);
		amps_4 = -saturate(amps_4, -sat, sat);
		amps_5 = -saturate(amps_5, -sat, sat);
		amps_6 = -saturate(amps_6, -sat, sat);


		//torque2 = saturate(torque2, sat);

		amps_[0].set_current(amps_0);
		amps_[1].set_current(amps_1);
		amps_[2].set_current(amps_2);
		amps_[3].set_current(amps_3);
		amps_[4].set_current(amps_4);
		amps_[5].set_current(amps_5);
		amps_[6].set_current(amps_6);

		// update hardware
		q8_.update_output();

		// wait for remainder of sample period
		timer.wait();

		std::cout << "Made a loop through the FOR" << std::endl;

		ms_Pos_Ref.write_data({ setpoint_0, amps_2, amps_3, amps_4, get_joint(0).get_position(), get_joint(4).get_velocity() });
		if (Keyboard::is_key_pressed(Key::Escape)) {
			break;
		}
	} //END FOR LOOP

	std::cout << "Broke out of the FOR" << std::endl;
	// check for exit key
	if (Keyboard::is_key_pressed(Key::Escape)) {
		g_stop = true;
	}

}
*/

void SparGlove::step_home() {
	std::cout << "I'm here" << std::endl;
	//Q8Usb q8;

	q8_.open();


	q8_.DO.set_enable_values(std::vector<Logic>(8, High));
	q8_.DO.set_disable_values(std::vector<Logic>(8, High));
	q8_.DO.set_expire_values(std::vector<Logic>(8, High));
	q8_.AO.set_enable_values(std::vector<double>(8, 0.0));
	q8_.AO.set_disable_values(std::vector<double>(8, 0.0));
	q8_.AO.set_expire_values(std::vector<Voltage>(8, 0.0));

	q8_.enable();

	Timer timer(hertz(1000));
	Time t;

	MelShare ms("motor_spin");

	q8_.encoder[0].zero();
	q8_.encoder[1].zero();
	//enc.set_units_per_count(2 * PI / 512.0);

	//Amplifier amp("a0", Low, q8_.DO[0], 1.8, q8_.AO[0]);
	//Motor motor("m0", 0.0109, amps_[0]);
	//Joint joint("j0", &motors_[0], &enc, &enc, 1 / 4.4 * 2 / (2 * PI));
	//(Actuator, PositionSensor, PSTransmission, VelocitySensor, VSTransmission, PosLimits ...)
	double ref = 0.0;
	double ref2 = 0.0;

	//PdController pd(0.2, 0.01);
	q8_.watchdog.start();

	while (!g_stop)
	{


		//std::cout << "got home" << std::endl;
		q8_.update_input();
		q8_.watchdog.kick();

		double act = get_joint(0).get_position();
		double act2 = get_joint(1).get_position();

		if (Keyboard::is_key_pressed(Key::E)) {
			motors_[0].enable();
			//	motors_[1].enable();
		}
		else if (Keyboard::is_key_pressed(Key::D)) {
			motors_[0].disable();
			motors_[1].disable();
		}

		if (Keyboard::is_key_pressed(Key::Up)) {
			ref += 0.01;
			ref2 += 0.01;
		}
		else if (Keyboard::is_key_pressed(Key::Down)) {
			ref -= 0.01;
			ref2 -= 0.01;
		}

		double amps = pd_controllers_[0].calculate(ref, act, 0, get_joint(0).get_velocity());
		double amps2 = pd_controllers_[1].calculate(ref, act2, 0, get_joint(1).get_velocity());
		amps = -saturate(amps, -0.17, 0.17);
		amps2 = -saturate(amps2, -0.17, 0.17);

		if (std::abs(amps) > 0.15) {
			g_stop = true;
		}

		amps_[0].set_current(amps);
		amps_[1].set_current(amps2);
		ms.write_data({ ref, act, act2, amps, amps2 });

		q8_.update_output();
		t = timer.wait();

		// check for exit key
		if (Keyboard::is_key_pressed(Key::Escape)) {
			g_stop = true;
			//amps_[0].set_current(0.0);
		}
	}

	//motors_[0].disable();
	//prompt("Amp should be disabled");
	q8_.disable();
	q8_.close();

}

/// Overrides the default Robot::enable function with some custom logic
bool SparGlove::on_enable() {
	//std::cout << "SparGlove is TOO enabled" << std::endl;
	q8_.open();
	mel::prompt("We are now in the on_enable script.  Press ENTER to open and enable Q8 USB.");

	q8_.DO.set_enable_values(std::vector<Logic>(8, High));
	q8_.DO.set_disable_values(std::vector<Logic>(8, High));
	q8_.DO.set_expire_values(std::vector<Logic>(8, High));
	q8_.AO.set_enable_values(std::vector<double>(8, 0.0));
	q8_.AO.set_disable_values(std::vector<double>(8, 0.0));
	q8_.AO.set_expire_values(std::vector<Voltage>(8, 0.0));

	//q8_.AO.set_disable_values(std::vector<Voltage>(8, 0.0)); // default is 0.0
	//q8_.AO.set_expire_values(std::vector<Voltage>(8, 0.0));  // default is 0.0

	//q8_.DO.set_disable_values(std::vector<Logic>(8, High)); // default is 0.0
	//q8_.DO.set_expire_values(std::vector<Logic>(8, High));  // default is 0.0

	q8_.enable();

	q8_.encoder[0].zero();
	q8_.encoder[1].zero();
	q8_.encoder[2].zero();
	q8_.encoder[3].zero();
	q8_.encoder[4].zero();
	q8_.encoder[5].zero();
	q8_.encoder[6].zero();

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

	motors_[0].disable();
	motors_[1].disable();
	motors_[2].disable();
	motors_[3].disable();
	motors_[4].disable();
	motors_[5].disable();
	motors_[6].disable();

	if (Robot::on_disable() && q8_.disable() && q8_.close())
	{


		return true;
	}
	else
		return false;

}