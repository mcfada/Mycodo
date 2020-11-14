# coding=utf-8
#
# grove_i2c_motor_driver.py - Output for Grove I2C
#

#Import stuff
import copy
import sys
import time
import threading

from flask_babel import lazy_gettext

from mycodo.databases.models import DeviceMeasurements
from mycodo.databases.models import OutputChannel
from mycodo.outputs.base_output import AbstractOutput
from mycodo.utils.database import db_retrieve_table_daemon
from mycodo.utils.influx import add_measurements_influxdb
from mycodo.utils.influx import read_last_influxdb


def constraints_pass_positive_value(mod_input,value):
  """
  Check if the user input is acceptable
  :param mod_input: SQL object with user-saved Input options
  :param value: float or int
  :return: tuple: (bool, list of strings)
  """
  errors = []
  all_passed = True
  # Ensure value is positive
  if value <= 0:
    all_passed = False
    errors.append("Must be a positive value")
  return all_passed, errors, mod_input

#Measurements - a dictionary of dictionaries defining the output measurements
measurements_dict = {
  0: {
      'measurement': 'duration_time',
      'unit': 's',
      'name': 'Pump On',
  },
  1: {
      'measurement': 'volume',
      'unit': 'ml',
      'name': 'Dispense Volume',
  },
  2: {
      'measurement': 'duration_time',
      'unit': 's',
      'name': 'Dispense Duration',
  },
  3: {
      'measurement': 'duration_time',
      'unit': 's',
      'name': 'Pump On',
  },
  4: {
      'measurement': 'volume',
      'unit': 'ml',
      'name': 'Dispense Volume',
  },
  5: {
      'measurement': 'duration_time',
      'unit': 's',
      'name': 'Dispense Duration',
  }
}

channels_dict = {
  0: {
      'name': 'Pump A',
      'types': ['volume', 'on_off'],
      'measurements': [0, 1]
  },
  1: {
      'name': 'Pump B',
      'types': ['volume', 'on_off'],
      'measurements': [2, 3]
  }
}


# Output information
OUTPUT_INFORMATION = {
  # A unique output name used to distinguish it from others
  'output_name_unique': 'grove_i2c_motor_driver',

  # A friendly/common name for the output to display to the user
  'output_name': "{} ({})".format(lazy_gettext('Dual Peristaltic Pump'), lazy_gettext('Grove')),

  # Optional library name (for outputs that are named the same but use different libraries)
  #'output_library': 'library_name',

  # The dictionary of measurements for this output. Don't edit this.
  'measurements_dict': measurements_dict,
  
  'channels_dict': channels_dict,

  # Type of output. Options: "on_off", "pwm", "volume"
  'output_types': ['volume','on_off'],
  
  'url_manufacturer': 'https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/',
  'url_datasheet': 'https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/',
  'url_product_purchase': 'https://www.amazon.com/dp/B07Q1C3PW2/ref=cm_sw_em_r_mt_dp_0QfSFbP395EAB?_encoding=UTF8&psc=1',

  # A message to display at the top of the output options
  'message': 'Grove I2C Motor Driver V1.3, used to power two peristaltic pumps by I2C. '
          'Pumps are available on Amazon and claim to dispense 5-100mL/min',
  
  # Form input options that are enabled or disabled
  'options_enabled': [
      'i2c_location',
      'button_on',        # Shows a button to turn the output on
      'button_send_volume',   # Shows an input field and a button to turn on for a volume
      'button_send_duration'  # Shows an input field and a button to turn on for a duration
  ],
  'options_disabled': [
      'interface'  # Show the interface (as a disabled input)
  ],
  
  # Any dependencies required by the output module
  'dependencies_module': [
      ('pip-pypi', 'smbus2', 'smbus2')
  ],
  
  # The interface or interfaces that can be used with this module
  # A custom interface can be used.
  # Options: SHELL, PYTHON, GPIO, I2C, FTDI, UART
  'interfaces': ['I2C'],
  
  'i2c_address_editable': True,
  'i2c_address_default': '0x0f',
  
  'custom_options_message':   "To accurately dispense specific volumes, set a target flow rate [ml/min] "
                                "and adjust the motor power [%] until the desired flow rate is reached. "
                                "Alternately, choose a motor power [%] and record the volume dispensed in 60 seconds. "
                                "A single flow rate set point is best to achieve accurate volumes, as many budget peristaltic "
                                "pumps do not have a linear relationship between flow rate and duty cycle. ",
  
  'custom_channel_options': [
      {
        'id': 'name',
        'type': 'text',
        'default_value': '',
        'required': False,
        'name': lazy_gettext('Name'),
        'phrase': lazy_gettext('A name for this channel')
      },
      {
        'id': 'flow_rate',
        'type': 'float',
        'default_value': 60.0,
        'constraints_pass': constraints_pass_positive_value,
        'name': lazy_gettext('Desired Flow Rate (ml/min)'),
        'phrase': lazy_gettext('Desired flow rate in ml/minute')
      },
      {
        'id': 'motor_power_percent',
        'type': 'float',
        'default_value': 60.0,
        'constraints_pass': constraints_pass_positive_value,
        'name': lazy_gettext('Motor Power (0-100%)'),
        'phrase': lazy_gettext('Power sent to motor to achieve desired flow rate')
      },
      {
        'id': 'reverse_pump',
        'type': 'bool',
        'default_value': False,
        'name': lazy_gettext('Reverse Direction'),
        'phrase': lazy_gettext('Mark to reverse the pump direction')
      },
  ],
}
  
  
class OutputModule(AbstractOutput):
  """
  An output support class that operates an output
  """
  def __init__(self, output, testing=False):
    super(OutputModule, self).__init__(output, testing=testing, name=__name__)

    self.currently_dispensing = [False,False]

    output_channels = db_retrieve_table_daemon(
      OutputChannel).filter(OutputChannel.output_id == self.output.unique_id).all()
    self.options_channels = self.setup_custom_channel_options_json(
            OUTPUT_INFORMATION['custom_channel_options'], output_channels)

  def setup_output(self):
    import smbus2

    self.setup_on_off_output(OUTPUT_INFORMATION)
    self.motor = None
    try:
      self.logger.debug("I2C: Address: {}, Bus: {}".format(self.output.i2c_location, self.output.i2c_bus))
      if self.output.i2c_location:
        self.motor = motor_driver(smbus2, self.output.i2c_bus, int(str(self.output.i2c_location), 16))
        self.output_setup = True
    except:
      self.logger.exception("Could not set up output")
      return

  def dispense_volume_rate(self, amount, dispense_rate, channel):
    
    total_dispense_seconds = amount / dispense_rate * 60
    self.logger.debug("Total duration to run: {0:.1f} seconds".format(total_dispense_seconds))
    
    self.currently_dispensing[channel] = True
    timer_dispense = time.time() + total_dispense_seconds
    
    while time.time() < timer_dispense and self.currently_dispensing[channel]:
      # On for duration
      self.logger.debug("Output turned on")
      
      #def MotorOn(self, motorSpeed, motorDirectionCCW, output_channel):
      self.motor.MotorOn(self.options_channels['motor_power_percent'][channel], self.options_channels['reverse_pump'][channel], channel)

    self.motor.MotorOn(0, self.options_channels['reverse_pump'][channel], channel)
    self.currently_dispensing[channel] = False
    self.logger.debug("Output turned off")
    self.record_dispersal(amount,total_dispense_seconds, channel)


  def record_dispersal(self, amount, total_dispense_seconds, channel):

    measure_dict = copy.deepcopy(measurements_dict)
    if not channel:
      measure_dict[0]['value'] = total_on_seconds
      measure_dict[1]['value'] = amount
    elif channel:
      measure_dict[2]['value'] = total_on_seconds
      measure_dict[3]['value'] = amount
    add_measurements_influxdb(self.unique_id, measure_dict)
    
  def output_switch(self, state, output_type=None, amount=None, output_channel=None):
    self.logger.debug("state: {}, output_type: {}, amount: {}".format(state, output_type, amount))

    for channel in channels_dict:
      if output_channel == channel:
        if self.options_channels['motor_power_percent'][channel] > 100:
          self.options_channels['motor_power_percent'][channel] = 100
        if state == 'on' and output_type == 'vol' and amount:
          if self.currently_dispensing[channel]:
            self.logger.debug("Pump instructed to turn on for a duration while it's already dispensing. "
                              "Overriding current dispense with new instruction.")
          dispense_rate = self.options_channels['flow_rate'][channel]
          self.logger.debug("Turning pump on to dispense {ml:.1f} ml at {rate:.1f} ml/min.".format(
                  ml=amount, rate=dispense_rate))
          write_db = threading.Thread(
            target=self.dispense_volume_rate,
            args=(amount, dispense_rate, channel,))
          write_db.start()
          return

        elif state == 'on':
          if not self.currently_dispensing[channel]:
            self.currently_dispensing[channel] = True
          motorSpeed = self.options_channels['motor_power_percent'][channel]
          self.motor.MotorOn(motorSpeed, self.options_channels['reverse_pump'][channel], channel)
          return
          
        elif state == 'off':
          if self.currently_dispensing[channel]:
            self.currently_dispensing[channel] = False
          self.logger.debug("Output turned off")
          self.motor.MotorOn(0, self.options_channels['reverse_pump'][channel], channel)
          return
        
        else:
          self.logger.error(
                "Invalid parameters: State: {state}, Type: {ot}, Amount: {amt}, Flow Rate: {fr}".format(
                    state=state,
                    ot=output_type,
                    amt=amount,
                    fr=self.options_channels['flow_rate'][channel]))
          return

  def is_on(self, output_channel=None):
    if self.is_setup():
      if output_channel is not None and output_channel in self.output_states:
        return self.currently_dispensing[output_channel]

  def is_setup(self):
    return self.output_setup


class motor_driver(object):

  MotorSpeedSet             = 0x82
  PWMFrequenceSet           = 0x84
  DirectionSet              = 0xaa
  MotorSetA                 = 0xa1
  MotorSetB                 = 0xa5
  Nothing                   = 0x01
  EnableStepper             = 0x1a
  UnenableStepper           = 0x1b
  Stepernu                  = 0x1c
  I2CMotorDriverAdd         = 0x0f  #Set the address of the I2CMotorDriver
  
  # /**************Motor Direction***************/
  BothClockWise             = 0x0a #0b1010
  BothAntiClockWise         = 0x05 #0b0101
  M1CWM2ACW                 = 0x06 #0b0110
  M1ACWM2CW                 = 0x09 #0b1001
  M1CW                      = 0x02 #0b0010
  M2CW                      = 0x08 #0b1000
  M1CCW                     = 0x01 #0b0001
  M2CCW                     = 0x04 #0b0100


  def __init__(self, smbus, i2c_bus, i2c_address):
    self.bus_no = i2c_bus
    self.bus = smbus.SMBus(i2c_bus)
    self.address = i2c_address
  
  def __repr__(self):
    return "motor_driver(i2c_bus_no=%r, address=0x%02x)" % (self.bus_no, self.address)

  #Maps speed from 0-100 to 0-255
  #value is a number you pass from 0-100, function returns an int from 0-255 to pass to motor
  
  def map_vals(self, value, leftMin, leftMax, rightMin, rightMax):
    '''http://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another'''
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return int(rightMin + (valueScaled * rightSpan))
  
  def MotorOn(self, motorSpeed, motorDirectionCCW, output_channel):

    Direction = None
    if motorDirectionCCW:
      if output_channel:
        Direction = self.M2CCW
      else: Direction = self.M1CCW
    elif output_channel:
      Direction = self.M2CW
    else: Direction = self.M1CW
    
    MotorSpeedA = 0
    MotorSpeedB = 0
    motorSpeed = self.map_vals(motorSpeed,0,100,0,255)
    if output_channel:
      MotorSpeedB = motorSpeed
    else: MotorSpeedA = motorSpeed
    
    self.bus.write_i2c_block_data(self.I2CMotorDriverAdd, self.MotorSpeedSet, [MotorSpeedA,MotorSpeedB])
    time.sleep(.02)
    self.bus.write_i2c_block_data(self.I2CMotorDriverAdd, self.DirectionSet, [Direction,0])
    time.sleep(.02)
