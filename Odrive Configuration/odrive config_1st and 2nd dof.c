// Factory reset Odrive
odrv0.erase_configuration()

//Check
odrv0.axis0.encoder.config.use_index_offset = True
odrv0.axis0.encoder.config.index_offset = 0

odrv0.axis1.encoder.config.use_index_offset = True
odrv0.axis1.encoder.config.index_offset = 0

//Motor Configuration for Turnigy 100 kV
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.motor.config.pole_pairs = 20
odrv0.axis0.motor.config.torque_constant = 8.27/100
odrv0.axis0.motor.config.calibration_current = 20

odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.motor.config.pole_pairs = 20
odrv0.axis1.motor.config.torque_constant = 8.27/100
odrv0.axis1.motor.config.calibration_current = 20

odrv0.axis0.motor.config.current_lim = 40
odrv0.axis0.controller.config.vel_limit = 5

odrv0.axis1.motor.config.current_lim = 40
odrv0.axis1.controller.config.vel_limit = 5

//Encoder configuration for AS5047D
odrv0.axis0.encoder.config.mode = 0
odrv0.axis0.encoder.config.cpr = 2000
odrv0.axis0.encoder.config.use_index = False

odrv0.axis1.encoder.config.mode = 0
odrv0.axis1.encoder.config.cpr = 2000
odrv0.axis1.encoder.config.use_index = False

//SPI Mode
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 3
odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 4
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis1.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis0.encoder.config.cpr = 2**14
odrv0.axis1.encoder.config.cpr = 2**14

//Startup configuration
//odrv0.axis0.config.startup_encoder_offset_calibration = True
odrv0.axis0.config.startup_closed_loop_control = False
odrv0.axis1.config.startup_closed_loop_control = False

odrv0.axis0.config.startup_encoder_index_search = False
odrv0.axis1.config.startup_encoder_index_search = False

//General configuration
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 2
odrv0.config.dc_bus_overvoltage_trip_level = 25

//Tuning
odrv0.axis0.controller.config.pos_gain = 100
odrv0.axis0.controller.config.vel_gain = 0.3
odrv0.axis0.controller.config.vel_integrator_gain = 0.4

odrv0.axis1.controller.config.pos_gain = 100
odrv0.axis1.controller.config.vel_gain = 0.3
odrv0.axis1.controller.config.vel_integrator_gain = 0.4

odrv0.axis0.trap_traj.config.vel_limit = 5.0
odrv0.axis0.trap_traj.config.accel_limit = 5.0
odrv0.axis0.trap_traj.config.decel_limit = 5.0

odrv0.axis1.trap_traj.config.vel_limit = 5.0
odrv0.axis1.trap_traj.config.accel_limit = 5.0
odrv0.axis1.trap_traj.config.decel_limit = 5.0

//Save and reboot
odrv0.save_configuration()
odrv0.reboot()


//Run encoder offset calibration
odrv0.axis0.requested_state = 3
odrv0.axis1.requested_state = 3

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.encoder.set_linear_count(0)
odrv0.axis1.encoder.set_linear_count(0)

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True

odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True

odrv0.axis0.controller.input_pos = 4*35/360
odrv0.axis1.controller.input_pos = -4*35/360

odrv0.axis0.controller.input_pos = 0
odrv0.axis1.controller.input_pos = 0


odrv0.axis0.requested_state = 1
odrv0.axis1.requested_state = 1

odrv0.axis0.encoder.pos_estimate
odrv0.axis1.encoder.pos_estimate

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.input_vel = 0.2

odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.input_vel = 0.2


odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ

//Check for motor error before setting pre_calibration
odrv0.axis0.motor.error


//Position control
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.input_pos


//Anti cogging Algorithm

//Start the calibration
odrv0.axis0.controller.start_anticogging_calibration()
odrv0.axis1.controller.start_anticogging_calibration()

//Put the gain back to what it was before
//odrv0.axis0.controller.config.vel_integrator_gain  = (0.5 * 10 * 0.32/2 )  //0.30
//odrv0.axis0.controller.config.pos_gain = 15 //100

// Wait until odrv0.axis0.controller.config.anticogging.calib_anticogging == False
// you can type 
odrv0.axis0.controller.config.anticogging.calib_anticogging
odrv0.axis1.controller.config.anticogging.calib_anticogging
// and just up arrow and enter till it turns false

//After the calibration is completed
odrv0.axis0.controller.config.anticogging.pre_calibrated = True
odrv0.axis1.controller.config.anticogging.pre_calibrated = True

odrv0.save_configuration()
odrv0.reboot()

//Set Linear Count with gear engaged
odrv0.axis0.encoder.set_linear_count(0)


//Old Odrives Settings for comparison:
odrv0.axis0.motor.config.motor_type = 0
odrv0.axis0.motor.config.pole_pairs = 20
odrv0.axis0.motor.config.torque_constant = 0.04
odrv0.axis0.motor.config.calibration_current = 10
odrv0.axis0.motor.config.current_lim = 40
odrv0.axis0.controller.config.vel_limit = 2

odrv0.axis0.encoder.config.mode = 0
odrv0.axis0.encoder.config.cpr = 2000

//Backup config
 C:\Users\HP\AppData\Local\Temp\odrive-config-206A389B304E.json

//Write output to file
def write(input1,input2,input3,input4,input5,file_name):
    f = open(file_name,'a')
    f.write(input1)
    f.write('\t')
    f.write(input2)
    f.write('\t')
    f.write(input3)
    f.write('\t')
    f.write(input4)
    f.write('\t')
    f.write(input5)
    f.write('\n')
    f.close()

for i in range(15000):
    Id1 = odrv0.axis0.motor.current_control.Id_measured
    Iq1 = odrv0.axis0.motor.current_control.Iq_measured
    Vd1 = odrv0.axis0.motor.current_control.v_current_control_integral_d
    Vq1 = odrv0.axis0.motor.current_control.v_current_control_integral_q

    Id2 = odrv0.axis1.motor.current_control.Id_measured
    Iq2 = odrv0.axis1.motor.current_control.Iq_measured
    Vd2 = odrv0.axis1.motor.current_control.v_current_control_integral_d
    Vq2 = odrv0.axis1.motor.current_control.v_current_control_integral_q

    P1 = Id1*Vd1 + Iq1*Vq1
    P2 = Id2*Vd2 + Iq2*Vq2
    write(str(Id1),str(Iq1),str(Vd1),str(Vq1),str(P1),'motor_axis0_rerun_18A.txt')
    write(str(Id2),str(Iq2),str(Vd2),str(Vq2),str(P2),'motor_axis1_rerun_18A.txt')
end 

Time for reading and writing data to txt : 0.00654 s
New time : 0.03984475135803223 s
