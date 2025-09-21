# PurgeBelt module for klipper
#
# Copyright (C) 2025 Eytecz
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import logging
import string

class PurgeBelt:
    def __init__(self, config):
        self.config = config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Read config from [purgebelt] section
        self.park_pos_x = config.getfloat('park_pos_x', 10.)                                   
        self.park_pos_y = config.getfloat('park_pos_y', 10.)                                   
        self.park_pos_z = config.getfloat('park_pos_z', 2.)                           
        self.purge_belt_z = config.getfloat('purge_belt_z', 0.)                        
        self.filament_diameter = config.getfloat('filament_diameter', 1.75, above=0)   
        self.purge_length = config.getfloat('purge_length', 50., above=0)
        self.purged_length_section_max = config.getfloat('purged_length_section_max', 200., above=0)         
        self.purge_layer_height = config.getfloat('purge_layer_height', 0.4, above=0.)          
        self.purge_extrusion_width = config.getfloat('purge_extrusion_width', 0.4, above=0.)    
        self.purge_belt_outfeed_length = config.getfloat('purge_belt_outfeed_length', 100., above=0)
        self.purge_belt_retract_travel_dist = config.getfloat('purge_belt_retract_travel_dist', 10., above=0)
        self.extrude_speed = config.getfloat('extrude_speed', 5., above=0)
        self.travel_speed = config.getfloat('travel_speed', 100., above=0)
        self.approach_speed = config.getfloat('approach_speed', 5., above=0)
        self.retract_speed = config.getfloat('retract_speed', 20., above=0)
        self.retract_dist = config.getfloat('retract_dist', 5., above=0)
        self.pause_time = config.getfloat('pause_time', 1., above=0)
        self.pause_qty = config.getint('pause_qty', 1)
        self.return_to_start_pos = config.getboolean('return_to_start_pos', True)
        self.max_flow_rate = config.getfloat('max_flow_rate', 40., above=0)
        self.flow_rate = config.getfloat('flow_rate', 30., above=0)

        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register g-code commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('SYNC_PURGE_BELT', self.cmd_SYNC_PURGE_BELT,
                                    desc = "Synchronizes the purge belt to the active extruder")
        self.gcode.register_command('UNSYNC_PURGE_BELT', self.cmd_UNSYNC_PURGE_BELT,
                                    desc = "Unsynchronizes the purge belt")
        self.gcode.register_command('PURGE_WITH_BELT', self.cmd_PURGE_WITH_BELT, desc="Run purge belt cycle")

        # Internal state
        self.sync_status = False
        self.purge_belt_stepper = None
        self.ea_index = None

    def handle_ready(self):
        self.min_event_systime = self.reactor.monotonic()

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')

        for manual_stepper in self.printer.lookup_objects('manual_stepper'):
            rail_name = manual_stepper[1].get_steppers()[0].get_name()
            if rail_name == 'manual_stepper purge_belt_stepper':
                self.purge_belt_stepper = manual_stepper[1]
        if self.purge_belt_stepper is None:
            raise self.printer.config_error("manual_stepper purge_belt_stepper must be specified")
    
    # Calculate new purge belt rotation distance when synchronized to the extruder
    def calc_purge_belt_rotation_distance_synced(self, layer_height, extrusion_width):
        extruder_flow_area = (math.pi / 4) * self.filament_diameter ** 2
        purge_belt_flow_correction_factor = extruder_flow_area / (layer_height * extrusion_width)
        purge_belt_rotation_distance_initial = self.purge_belt_stepper.get_steppers()[0].get_rotation_distance()[0]

        # When synched, klipper automatically wants to extrude an equal amount of material, however when synched, the goal
        # is to actually run the belt faster/slower based on the desired layer height and width. However, we let klipper
        # think it is still 'matched' to before/after extruding
        purge_belt_rotation_distance_synced = purge_belt_rotation_distance_initial / purge_belt_flow_correction_factor
        return purge_belt_rotation_distance_synced

    def cmd_SYNC_PURGE_BELT(self, gcmd):
        layer_height = gcmd.get_float('LAYER_HEIGHT', self.purge_layer_height)
        extrusion_width = gcmd.get_float('EXTRUSION_WIDTH', self.purge_extrusion_width)
        self.sync_purge_belt(layer_height, extrusion_width)

    def cmd_UNSYNC_PURGE_BELT(self, gcmd):
        self.unsync_purge_belt()
    
    def sync_purge_belt(self, layer_height, extrusion_width):
        if self.sync_status:
            pass
        else:
            self.last_purge_belt_rotation_distance = self.purge_belt_stepper.get_steppers()[0].get_rotation_distance()[0]
            rotation_dist = self.calc_purge_belt_rotation_distance_synced(layer_height, extrusion_width)
            self.purge_belt_stepper.get_steppers()[0].set_rotation_distance(rotation_dist)
            ea = self.get_available_ea_label()
            self.gcode.run_script_from_command(
                f'MANUAL_STEPPER STEPPER=purge_belt_stepper GCODE_AXIS={ea}')
            self.ea_index = self.get_ea_index(ea)
            logging.info(f'Synchronized purge belt with extruder on axis {ea} with index {self.ea_index}')
            self.sync_status = True

    def unsync_purge_belt(self):
        if not self.sync_status:
            pass
        else:
            self.gcode.run_script_from_command(
                f'MANUAL_STEPPER STEPPER=purge_belt_stepper GCODE_AXIS=')
            self.purge_belt_stepper.get_steppers()[0].set_rotation_distance(self.last_purge_belt_rotation_distance)
            self.sync_status = False

    def get_available_ea_label(self):
        axis_map = self.gcode_move.axis_map
        used_axes = set(axis_map.keys())
        for letter in string.ascii_uppercase:
            if letter in ("X", "Y", "Z", "E", "F", "N"):
                continue
            if letter not in used_axes:
                return letter
        raise ValueError("No free axis label found")
    
    def get_ea_index(self, ea):
        ea_index = self.gcode_move.axis_map[f"{ea}"]
        if ea_index is None:
            raise ValueError(f"Axis label {ea} is not available")
        return ea_index

    def get_init_toolhead_pos(self):
        init_toolhead_pos = self.toolhead.get_position()
        self.gcode_move.last_position = init_toolhead_pos
        return init_toolhead_pos

    def cmd_PURGE_WITH_BELT(self, gcmd):
        layer_height = gcmd.get_float('LAYER_HEIGHT', self.purge_layer_height)
        extrusion_width = gcmd.get_float('EXTRUSION_WIDTH', self.purge_extrusion_width)
        purge_volume = gcmd.get_float('PURGE_VOLUME', None)
        if purge_volume:
            purge_length = purge_volume / ((math.pi / 4) * (self.filament_diameter ** 2))
        else:
            purge_length = gcmd.get_float('PURGE_LENGTH', self.purge_length)
            purge_volume = ((math.pi / 4) * (self.filament_diameter ** 2)) * purge_length
        purged_length = purge_volume / (extrusion_width * layer_height)
        pause_qty = gcmd.get_int('PAUSE_QTY', None)
        if pause_qty is None and purged_length > self.purged_length_section_max:
            pause_qty = int(math.ceil(purged_length / self.purged_length_section_max))
        else:
            pause_qty = gcmd.get_int('PAUSE_QTY', self.pause_qty)
        flow_rate = gcmd.get_float('FLOW_RATE', None)
        if flow_rate is not None:
            if flow_rate > self.max_flow_rate:
                flow_rate = self.max_flow_rate
            extrusion_speed = flow_rate / ((math.pi / 4) * (self.filament_diameter ** 2))
        else:
            extrusion_speed = gcmd.get_float('EXTRUSION_SPEED', None)
            if extrusion_speed is not None:
                extrusion_speed = self.extrude_speed
                flow_rate = extrusion_speed * ((math.pi / 4) * (self.filament_diameter ** 2))
            else:
                flow_rate = self.flow_rate
                extrusion_speed = flow_rate / ((math.pi / 4) * (self.filament_diameter ** 2))
        pause_time = gcmd.get_float('PAUSE_TIME', self.pause_time)
        self.gcode.respond_info(f"Purging {purge_volume:.2f} mm³ at {flow_rate:.2f} mm³/s in {pause_qty + 1} step(s)")
        self.purge_cycle(purge_length, layer_height, extrusion_width, extrusion_speed, pause_qty, pause_time)

    def move_to_park_pos(self):
        # Get initial position to return to
        self.init_pos = self.get_init_toolhead_pos()

        # Travel to park location
        self.gcode_move.last_position = [self.park_pos_x, self.park_pos_y, self.park_pos_z, self.init_pos[3]]
        self.toolhead.move(self.gcode_move.last_position, self.travel_speed)
        self.toolhead.wait_moves()
    
    def restore_pos(self):
        # Return to initial position
        if self.return_to_start_pos:
            self.init_pos[3] = self.gcode_move.last_position[3]
            self.gcode.last_position = self.init_pos
            self.toolhead.move(self.gcode_move.last_position, self.travel_speed)
            self.toolhead.wait_moves()
    
    def purge_cycle(self, purge_length, layer_height, extrusion_width, extrusion_speed, pause_qty, pause_time):
        # Travel to park position
        self.move_to_park_pos()

        # Pre-calculate purge height
        purge_height = self.purge_belt_z + layer_height

        # Purge with or without pauses
        if pause_qty == 0:
            # Travel to purge height above belt depending on layer height
            self.gcode_move.last_position[2] = purge_height
            self.toolhead.move(self.gcode_move.last_position, self.approach_speed) 
            self.toolhead.wait_moves()      

            # Do the actual purging
            self.sync_purge_belt(layer_height, extrusion_width)
            self.gcode_move.last_position[3] += purge_length
            self.gcode_move.last_position[self.ea_index] += purge_length
            self.toolhead.move(self.gcode_move.last_position, extrusion_speed)
            self.toolhead.wait_moves()
            self.unsync_purge_belt()

            # Retract filament and keep belt running for an additional distance
            self.gcode_move.last_position[3]-= self.retract_dist
            movepos = self.purge_belt_stepper.get_position()[0] + self.purge_belt_retract_travel_dist
            self.purge_belt_stepper.do_move(movepos, self.purge_belt_stepper.velocity, self.purge_belt_stepper.accel, sync=True)
            self.toolhead.move(self.gcode_move.last_position, self.retract_speed)
            self.toolhead.wait_moves()
            
            # Return to parking position
            self.gcode_move.last_position[2] = self.park_pos_z
            self.toolhead.move(self.gcode_move.last_position, self.travel_speed)
            self.toolhead.wait_moves()
        else:
            purge_length = purge_length / (pause_qty + 1)
            for i in range(pause_qty + 1):
                # Travel to purge height above belt depending on layer  height
                self.gcode_move.last_position[2] = purge_height
                self.toolhead.move(self.gcode_move.last_position, self.approach_speed) 
                self.toolhead.wait_moves()

                # Do the actual purging
                self.sync_purge_belt(layer_height, extrusion_width)
                self.gcode_move.last_position[3] += purge_length
                self.gcode_move.last_position[self.ea_index] += purge_length
                self.toolhead.move(self.gcode_move.last_position, extrusion_speed)
                self.toolhead.wait_moves()
                self.unsync_purge_belt()

                # Retract filament and keep belt running for an additional distance
                self.gcode_move.last_position[3]-= self.retract_dist
                movepos = self.purge_belt_stepper.get_position()[0] + self.purge_belt_retract_travel_dist
                self.purge_belt_stepper.do_move(movepos, self.purge_belt_stepper.velocity, self.purge_belt_stepper.accel, sync=True)
                self.toolhead.move(self.gcode_move.last_position, self.retract_speed)
                self.toolhead.wait_moves()

                # Return to parking position
                self.gcode_move.last_position[2] = self.park_pos_z
                self.toolhead.move(self.gcode_move.last_position, self.travel_speed)
                self.toolhead.wait_moves()

                # Dwell for the defined pause time if not last iteration
                if i < pause_qty:
                    delay = pause_time
                    self.toolhead.dwell(delay)
        
        # Clear belt from filament
        pos_belt = self.purge_belt_stepper.get_position()[0]
        movepos = pos_belt + self.purge_belt_outfeed_length
        self.purge_belt_stepper.do_move(movepos, self.purge_belt_stepper.velocity, self.purge_belt_stepper.accel, sync=True)
        self.toolhead.wait_moves()

        # Return to start position
        self.restore_pos()




def load_config(config):
  return PurgeBelt(config)