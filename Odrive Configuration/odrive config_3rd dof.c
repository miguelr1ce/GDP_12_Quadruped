// Factory reset Odrive
odrv0.erase_configuration()

//check
odrv0.axis0.encoder.config.use_index_offset
odrv0.axis0.encoder.config.index_offset

odrv0.axis1.encoder.config.use_index_offset
odrv0.axis1.encoder.config.index_offset

//Motor Configuration for C5065 435 kV
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.torque_constant = 8.27/435
odrv0.axis0.motor.config.calibration_current = 20

odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.motor.config.pole_pairs = 7
odrv0.axis1.motor.config.torque_constant = 8.27/435
odrv0.axis1.motor.config.calibration_current = 20

odrv0.axis0.motor.config.current_lim = 40
odrv0.axis0.controller.config.vel_limit = 5

odrv0.axis1.motor.config.current_lim = 40
odrv0.axis1.controller.config.vel_limit = 5

odrv0.axis0.encoder.config.mode = 0
odrv0.axis0.encoder.config.cpr = 4000
odrv0.axis0.encoder.config.use_index = False

odrv0.axis1.encoder.config.mode = 0
odrv0.axis1.encoder.config.cpr = 4000
odrv0.axis1.encoder.config.use_index = False

odrv0.axis0.controller.config.pos_gain = 40
odrv0.axis0.controller.config.vel_gain = 0.125
odrv0.axis0.controller.config.vel_integrator_gain = 0.2

odrv0.axis1.controller.config.pos_gain = 40
odrv0.axis1.controller.config.vel_gain = 0.125
odrv0.axis1.controller.config.vel_integrator_gain = 0.2
