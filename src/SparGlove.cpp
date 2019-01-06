#include "SparGlove.hpp"



SparGlove::SparGlove() :
	
	Robot("spar_glove"),
	q8_(),
	kt_{1,2,3,4,5,6,7},
	motor_cont_limits_{1,2,3,4,5,6,7},  //amps
	motor_peak_limits_{ 1,2,3,4,5,6,7 }, //amps
	motor_i2t_times_{ seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1) },
	eta_{ 1,2,3,4,5,6,7 },
	encoder_res_{ 1,2,3,4,5,6,7 },
	pos_limits_neg_{ 1,2,3,4,5,6,7 },
	pos_limits_pos_{ 1,2,3,4,5,6,7 },
	vel_limits_{ 1,2,3,4,5,6,7 },
	joint_torque_limits{ 1,2,3,4,5,6,7 },
	kin_friction_{ 1,2,3,4,5,6,7 },
	pd_controllers_(7),
	encoders_(7)
{
	//motor_cont_limits_ = std::array<double, 7>({ 1,2,3,4,5,6,7 });
	//motor_peak_limits_[0] = 1;
	//motor_peak_limits_[1] = 2;
	// ...

	// Components
	amps_.reserve(7);
	amps_.emplace_back("amp1", Low, q8_.DO[0], 1.0, q8_.AO[0]); // THERE ARE MORE ARGS AVAILABLE!
	amps_.emplace_back("amp2", Low, q8_.DO[1], 1.0, q8_.AO[1]); // THERE ARE MORE ARGS AVAILABLE!
	amps_.emplace_back("amp3", Low, q8_.DO[2], 1.0, q8_.AO[2]);
	amps_.emplace_back("amp4", Low, q8_.DO[3], 1.0, q8_.AO[3]);
	amps_.emplace_back("amp5", Low, q8_.DO[4], 1.0, q8_.AO[4]);
	amps_.emplace_back("amp6", Low, q8_.DO[5], 1.0, q8_.AO[5]);
	amps_.emplace_back("amp7", Low, q8_.DO[6], 1.0, q8_.AO[6]);


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


	pd_controllers_[0] = PdController(2, 3);
	pd_controllers_[1] = PdController(2, 3);
	// do 6 more times

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

SparGlove::SparGlove(const SparGlove&):
	Robot("spar_glove"),
	q8_(),
	kt_{ 1,2,3,4,5,6,7 },
	motor_cont_limits_{ 1,2,3,4,5,6,7 },  //amps
	motor_peak_limits_{ 1,2,3,4,5,6,7 }, //amps
	motor_i2t_times_{ seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1), seconds(1) },
	eta_{ 1,2,3,4,5,6,7 },
	encoder_res_{ 1,2,3,4,5,6,7 },
	pos_limits_neg_{ 1,2,3,4,5,6,7 },
	pos_limits_pos_{ 1,2,3,4,5,6,7 },
	vel_limits_{ 1,2,3,4,5,6,7 },
	joint_torque_limits{ 1,2,3,4,5,6,7 },
	kin_friction_{ 1,2,3,4,5,6,7 },
	pd_controllers_(7),
	encoders_(7)
{

}

SparGlove::~SparGlove() {
	if (is_enabled())
		disable();
}

void SparGlove::notify(std::size_t pred_label) {
	//Functions that need to know about pred_label
	//Experiment1
	experiment1(std::atomic_bool( false ), pred_label);

	//GloveDriver
}

void SparGlove::experiment1(std::atomic<bool>& stop, size_t pred_label) {

	//q8_.enable();

	// create control loop timer
	Timer timer(hertz(1000));
	
	MelShare ms("motor_position");
	std::vector<double> to_ms_data(4);
	
	// enter control loop
	prompt("Press ENTER to start control loop");
	while (!stop) {
		

		std::cout << "But SparGlove thinks pred_label is" << std::endl;

		//Prove that we can see pred_label
		std::cout << pred_label << std::endl;

		// update hardware
		q8_.update_input();


		//	std::array<double, 7> sat_torques = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; // temporary saturation torques
		double sat = 0.1;

		// experiment 1 code
		double pos_ref = get_joint(1).get_position();
		double pos_act = get_joint(0).get_position();
		double vel_act = get_joint(0).get_velocity();
		double torque2 = pd_controllers_[0].calculate(pos_ref, pos_act, 0, vel_act);
		double torque = 0.05;
		double nothing = 0.0;
		torque = saturate(torque, sat);
		torque2 = saturate(torque2, sat);

		get_joint(3).set_torque(torque);
		get_joint(0).set_torque(nothing);

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
}

/// Overrides the default Robot::enable function with some custom logic
bool SparGlove::on_enable() {
	std::cout << "SparGlove is TOO enabled" << std::endl;
	q8_.open();
	mel::prompt("Press ENTER to open and enable Q8 USB.");
	q8_.enable();
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
		return true;
	else
		return false;

}