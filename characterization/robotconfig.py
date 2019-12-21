{
  # Ports for the left-side motors
  'leftMotorPorts': [20, 21],
  # Ports for the right-side motors
  'rightMotorPorts': [22, 23],

  # NOTE: Inversions of the slaves (i.e. any motor *after* the first on
  # each side of the drive) are *with respect to their master*.  This is
  # different from the other poject types!
  # Inversions for the left-side motors
  'leftMotorsInverted': [True, False],
  # Inversions for the right side motors
  'rightMotorsInverted': [True, False],

  # The total gear reduction between the motor and the wheels, expressed as
  # a fraction [motor turns]/[wheel turns]
  'gearing': 1 / (9/84),
  # Wheel diameter (in units of your choice - will dictate units of analysis)
  'wheelDiameter': 0.15494,

  # Whether the robot should turn (for angular tests)
  'turn': False,
}

