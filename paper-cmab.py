'''
This is a simulation demonstrating the performance benefit of using MAB 
and C-MAB for mmWave small cell V2X application.

In this example, we use base station (BS) centric where a BS will send a hello message
to collect cqi from all vehicles, and then associate with the vehicle with various 
methods including (i) random, (ii) the highest cqi or best SNR, (iii) MAB, (iv) C-MAB. 
In our system, cqi is the received power (in dBm).

This code is designed to run in PyMoSim v0.8.0.
'''

import wx
import operator
import random
import math
import json

## pymosim packages
from sim.simulation import World
from sim.loc import ScreenXY as XY
from sim.scenario import BaseScenario
from sim.event import Event
from node.node import BaseNode
from node.mobility import Stationary, StaticPath
from comm.mmwave28 import Transceiver28GHz, Transceiver28GHzSteered
from comm.mmwave28 import Channel28GHz
from map.mapinfo import MapInfo

####################################################################
## Communication Module
####################################################################

class CommModule:
    def __init__(self, node):
        self._node = node

    def send_hello_to(self, other):
        '''This method triggers a greeting handshake between this object and
        `other`.

        Returns
        -------
        Tuple (outcome:bool, cqi:float)
            It return the `outcome` (True/False) whether the hello and hello-reply
            can be received and detected successfully. If True, `cqi` contains the 
            channel state indicator (cqi) of hello-reply packet from `other`.
            In our system, we use received power (dBm) as cqi directly.
        '''
        me = self._node
        self_transceiver = self._node.get("transceiver")
        other_transceiver = other.get("transceiver")

        # send hello-message
        hello_message = self_transceiver.create_signal()
        recv_signal = self_transceiver.unicast(hello_message, other)
        if recv_signal is None:
            return (False, None) # signal can't reach other? return False

        # receiver replies with hello-reply
        hello_reply = other_transceiver.create_signal()
        recv_signal = other_transceiver.unicast(hello_reply, me)
        if recv_signal is None:
            return (False, None) # reply can't reach me? return False

        # return True and rx_power
        return (True, recv_signal.quality)

####################################################################
## Base station & Vehicle
####################################################################

class MyBaseStationBeam(BaseNode):
    '''
    MyBaseStationBeam: This is a beam of a base station.
    '''
    def __init__(self, simworld, id, loc, freq, channel, sector_width, sector_dir):
        super().__init__(simworld, id, node_type=BaseNode.Type.BS)

        ## create transceiver
        ## convert sector_width to beam_3dB
        #  for 6 sectors, sector_width=60.  Based on 3GPP, beam_3dB = 35
        #  for 3 sectors, sector_width=120. Based on 3GPP, beam_3dB = 70
        #  so, beam_3dB = sector_width * 35/60 degree (to convert to rad)
        beam_3dB_rad = sector_width * 35/60 * math.pi/180

        ## setup the transceiver to create coverage range around 80
        self.transceiver = Transceiver28GHz(self, freq, channel,
                                    bandwidth=50e6, # Hz
                                    tx_power=30, # dBm
                                    gain_tx=14,  # dB
                                    gain_rx=0,   # dB
                                    snr_threshold= -5, # dB
                                    noise_figure=4, # dB
                                    beam_3dB=beam_3dB_rad, 
                                    pointing_dir=sector_dir)

        ## setup the sector beam
        self.set_transceiver(self.transceiver)
        self.set_mobility(Stationary(loc))
        self.comm = CommModule(self)
        self.serving_node = None

    ## inform this beam about a lost of connection
    def lost_vehicle(self, sim_time):
        self.serving_node.lost_bs(sim_time)
        self.serving_node = None

    ## inform this beam about a new connection
    def connect_vehicle(self, vehicle, sim_time, cqi):
        self.serving_node = vehicle
        self.serving_node.connect_bs(self, sim_time)

    ## show the coverage of this beam (illustration only, not actual coverage)
    def show_coverage(self):

        self.clear_drawing() # this is persistent drawing, so need to clear the all first

        if self.serving_node==None:
            pen = wx.Pen(wx.RED,2,style=wx.PENSTYLE_SHORT_DASH)
            brush = wx.TRANSPARENT_BRUSH
        else: # serving a vehicle
            pen = wx.Pen(wx.BLACK,4,style=wx.PENSTYLE_SOLID)
            brush = wx.Brush(wx.RED,style=wx.BRUSHSTYLE_BDIAGONAL_HATCH)

        self.draw_sector(self.transceiver.get_property("radius"),
                            self.transceiver.get_property("azimuth"),
                            self.transceiver.get_property("beam width"),
                            pen, brush)
        
##############################################################
class MyVehicle(BaseNode):
    '''
    MyVehicle: This is a vehicle design.
    '''
    def __init__(self, simworld, id, map, freq, channel):
        super().__init__(simworld, id, node_type=BaseNode.Type.Vehicle)

        ## create transceiver
        self.transceiver = Transceiver28GHzSteered(self, freq, channel,
                                    tx_power=20, # dBm
                                    gain_tx=14,  # dB
                                    gain_rx=0,   # dB
                                    snr_threshold= -5, # dB
                                    noise_figure=7) # dB

        ## setup the vehicle properties
        self.comm = CommModule(self)
        self.set_transceiver(self.transceiver)
        self.map = map
        self.associated_bs = None

        ## for visible beam recording
        self.visible_beam = None
        self.visible_beam_start = 0
        self.visible_beam_list = []

    ## set visible beam recording
    def set_visible_beam(self, sim_time, beam):
        if beam!=None: beam=str(beam.id)
        if self.visible_beam==None and beam!=None: # new beam?
            self.visible_beam = beam
            self.visible_beam_start = sim_time
        elif self.visible_beam!=None and beam==None: # lost beam
            conn_tuple = (self.visible_beam, sim_time-self.visible_beam_start)
            self.visible_beam_list.append(conn_tuple)
            self.visible_beam = None
        elif self.visible_beam!=None and beam!=None and self.visible_beam!=beam: # changed beam
            conn_tuple = (self.visible_beam, sim_time-self.visible_beam_start)
            self.visible_beam_list.append(conn_tuple)
            self.visible_beam = beam
            self.visible_beam_start = sim_time

    ## set a route for this vehicle
    def set_route(self, start_pin, end_pin, speed_low, speed_high, time_scale) -> bool:
        path = self.map.find_path(start_pin,end_pin)
        if len(path)==0: return False # can't find a path

        route = []
        x,y,_ = path[0] # unpack the starting point
        start_loc = XY(x,y) # create an XY point
        speed = random.uniform(speed_low,speed_high)            # in km/h
        speed = self.map.kph(speed, speed_up_factor=time_scale) # in pixels/sec
        for waypoint in path[1:]:
            route.append((speed,XY(waypoint[0],waypoint[1])))

        self.set_mobility(StaticPath(start_loc,route))
        self.end_pin = end_pin
        return True

    ## calculate the distance from the BS using timing advance
    def dist_by_timing_advance(self, bs):
        return self.get("location").distance_to(bs.get("location"))

    ## inform this node about a lost of connection
    def lost_bs(self, sim_time):
        self.associated_bs = None

    ## inform this node about a new connection establishment
    def connect_bs(self, bs, sim_time):
        self.associated_bs = bs

    ## show connection status on the animation
    def show_connection(self):
        self.clear_drawing()
        if self.associated_bs!=None:
            self.draw_line(self.associated_bs)
            self.set_color(wx.BLACK)
        else:
            self.set_color(wx.RED)

####################################################################
## Various vehicle selection algorithm
####################################################################

class VehicleSelectionAlgorithm:
    '''
    Base class, no algorithm is implemented.
    '''
    def __init__(self, scenario, name="no algorithm"):

        ## property
        self.name = name

        ## transferring scenario variables to own properties for easy access
        self.all_beams = scenario.bs_beams
        self.all_vehicles = scenario.all_vehicles
        self.max_active_sector = scenario.max_active_beam

        ## for stats
        class Stats:
            def __init__(self):
                self.conn_start_time = None
                self.total_time = 0 # total connection time
                self.total_conn = 0 # total number of connections
            def conn_begin(self, sim_time):
                self.conn_start_time = sim_time
            def conn_end(self, sim_time):
                self.total_time += sim_time - self.conn_start_time
                self.total_conn += 1
            def get_average_connection_time(self):
                if self.total_conn==0: return 0
                return self.total_time/self.total_conn
            def get_connection_count(self):
                return self.total_conn
            def reset(self):
                self.total_time = 0
                self.total_conn = 0
        self.stats = {}  # first statistics, from start to end for overall outcome
        self.stats2 = {} # second statistics, get reset periodically for sessional outcome
        for beam in self.all_beams:
            self.stats[beam] = Stats() # stats for each beam
            self.stats2[beam] = Stats() # stats for each beam

    def get_name(self):
        return self.name

    def do_pairing(self, sim_time):
        return None # should return the Beam-Vehicle pair, precisely: (beam,vehicle,cqi)

    ## report a connection establishment (for statistics)
    def report_conn_confirmed(self, sim_time, beam, vehicle, cqi):
        self.stats[beam].conn_begin(sim_time)
        self.stats2[beam].conn_begin(sim_time)

    ## report a connection lost (for statistics)
    def report_conn_lost(self, sim_time, beam, vehicle):
        self.stats[beam].conn_end(sim_time)
        self.stats2[beam].conn_end(sim_time)

##############################################################
class RandomSelection(VehicleSelectionAlgorithm):
    '''
    Random selecting a vehicle.
    '''

    def __init__(self, scenario, name="Random"):
        super().__init__(scenario, name)

    def do_pairing(self, sim_time):

        available_beams = [beam for beam in self.all_beams if beam.serving_node==None]

        while True:

            ## if no more available beam, done & return None; else pick one
            if len(available_beams)==0: return None
            beam = random.choice(available_beams)

            ## pick an available vehicle within the beam
            available_vehicle_list = []
            beacon = beam.get("transceiver").create_signal()
            vehicle_list = beam.get("transceiver").broadcast(beacon)
            for (vehicle,signal) in vehicle_list:
                ## check that the reachable node is a vehicle without service
                if vehicle.type!=BaseNode.Type.Vehicle: continue # skip if not Vehicle
                if vehicle.associated_bs!=None: continue # skip if already being served
                ## send hello message to obtain the cqi
                (is_successful, cqi) = beam.comm.send_hello_to(vehicle)
                if not is_successful: continue # skip if failed, likely not in coverage
                ## append to the detection list
                available_vehicle_list.append((vehicle,cqi))

            if len(available_vehicle_list)==0:
                available_beams.remove(beam) # no vehicle, don't choose it again
                continue                     # and try another beam
            else:
                (vehicle,cqi) = random.choice(available_vehicle_list)
                return (beam,vehicle,cqi)    # a vehicle is picked, done & return

##############################################################
class HighestCQI(VehicleSelectionAlgorithm):
    '''
    Selecting the highest CQI (or strongest SNR).
    '''

    def __init__(self, scenario, name="HighestCQI"):
        super().__init__(scenario, name)

    def do_pairing(self, sim_time):

        ## no more available beam?
        available_beams = [beam for beam in self.all_beams if beam.serving_node==None]
        if len(available_beams)==0: return None

        ## scan all vehicles in all available beams to find one with the highest cqi
        max_cqi = None
        selected_pair = None
        for beam in available_beams:
            beacon = beam.get("transceiver").create_signal()
            vehicle_list = beam.get("transceiver").broadcast(beacon)
            for (vehicle,signal) in vehicle_list:

                ## check that the reachable node is a vehicle without service
                if vehicle.type!=BaseNode.Type.Vehicle: continue # skip if not Vehicle
                if vehicle.associated_bs!=None: continue # skip if already being served

                ## vehicle to reply the beam
                beacon_reply = vehicle.transceiver.create_signal()
                recv_signal = vehicle.transceiver.unicast(beacon_reply,beam)
                if recv_signal is None: continue # skip if failed, likely not in coverage
                cqi = recv_signal.quality
                
                ## keep the pairing info carrying the highest cqi
                if max_cqi is None or cqi>max_cqi:
                    max_cqi = cqi
                    selected_pair = (beam,vehicle,cqi)

        return selected_pair

##############################################################
class CMAB(RandomSelection):
    '''
    Contextual MAB. It uses random selection for exploration, so 
    we inherit RandomSelection.
    '''

    def __init__(self, scenario, time_unit, name="CMAB"):
        super().__init__(scenario, name)

        self.arm_total_reward = {}  # for each context
        self.arm_total_count = {}   # for each context
        self.arm_info = {}    # lookup for pulled arm: [vehicle]=>(context,start_time)

        self.total_pulled = 0 # number of arms pulled so far
        self.time_unit = time_unit # time unit, used for reward calc

        self.exploration_stop_time = 0 # Epsilon-first strategy
        self.use_speed = False

    def mab_setting(self, stop_time=None, use_speed=None):
        if stop_time is not None:
            self.exploration_stop_time = stop_time
        if use_speed is not None:
            self.use_speed = use_speed

    def get_context(self, beam, vehicle, cqi):

        context_str = "[" + str(beam.id) + "; "

        if self.use_speed:
            speed = vehicle.get("speed")
            if speed<40: speed_str="slow"
            elif speed<50: speed_str="med "
            else: speed_str="fast"
            context_str += "speed=%s; "%speed_str

        dist = vehicle.dist_by_timing_advance(beam)
        if dist<180: dist_str="near" 
        elif dist<360: dist_str="med "
        else: dist_str="far "
        context_str += "dist=%s; "%dist_str

        dir = vehicle.get("direction").get_azimuth()
        dir_group = 4 # number of direction
        dir_width = 360/dir_group
        for i in range(dir_group):
            if dir<=dir_width*(i+1): 
                context_str += "dir=%02d]"%(i+1)
                break

        return context_str

    def do_pairing(self, sim_time):

        ## for exploration (Epsilon-first strategy)
        if sim_time<self.exploration_stop_time:
            pair = super().do_pairing(sim_time) # super_class is random selection
            self.total_pulled += 1 # another arm pulled
            return pair

        ## for exploitation
        else:
            ## no more available beam?
            available_beams = [beam for beam in self.all_beams if beam.serving_node==None]
            if len(available_beams)==0: return None

            ## scan all vehicles in all available beams to find one with the highest predicted reward
            highest_reward = -1
            selected_pair = None
            for beam in available_beams:
                beacon = beam.get("transceiver").create_signal()
                vehicle_list = beam.get("transceiver").broadcast(beacon)
                for (vehicle,signal) in vehicle_list:

                    ## check that the reachable node is a vehicle without service
                    if vehicle.type!=BaseNode.Type.Vehicle: continue # skip if not Vehicle
                    if vehicle.associated_bs!=None: continue # skip if already being served

                    ## vehicle to reply the beam
                    beacon_reply = vehicle.transceiver.create_signal()
                    recv_signal = vehicle.transceiver.unicast(beacon_reply,beam)
                    if recv_signal is None: continue # skip if failed, likely not in coverage
                    cqi = recv_signal.quality

                    ## keep the pairing info carrying the highest predicted reward based on context
                    context = self.get_context(beam,vehicle,cqi)
                    predicted_reward = self.get_past_reward(context)
                    if predicted_reward!=None:
                        if predicted_reward>highest_reward:
                            highest_reward = predicted_reward
                            selected_pair = (beam,vehicle,cqi)

            self.total_pulled += 1 # another arm pulled
            return selected_pair   # the pulled arm is recorded in `selected_pair`
            
    def get_past_reward(self, context):
        if context in self.arm_total_reward and self.arm_total_count[context]!=0:
            past_reward = self.arm_total_reward[context]/self.arm_total_count[context]
            past_reward *= self.time_unit # adjust to seconds
            return past_reward 
        else:
            return None

    def report_conn_confirmed(self, sim_time, beam, vehicle, cqi):

        super().report_conn_confirmed(sim_time, beam, vehicle, cqi)

        ## initialize the reward var if needed
        context = self.get_context(beam, vehicle, cqi)
        if context not in self.arm_total_reward:
            self.arm_total_reward[context] = 0
            self.arm_total_count[context] = 0

        ## keep connection info in `arm_info` to use for 
        ## reward update when this connection is lost
        self.arm_info[vehicle] = (context, sim_time)

    def report_conn_lost(self, sim_time, beam, vehicle):

        super().report_conn_lost(sim_time, beam, vehicle)

        ## update the reward for the connection, the
        ## connection is given in `arm_info`
        (context, start_time) = self.arm_info[vehicle]
        self.arm_total_reward[context] += sim_time - start_time
        self.arm_total_count[context] += 1

##############################################################
class MAB(CMAB):
    '''
    MAB is CMAB without context. Or it is essentially CMAB with a single context.
    So we inherit CMAB and implement a single context ignoring vehicle profiles.
    '''

    def __init__(self, scenario, time_unit, name="MAB"):
        super().__init__(scenario, time_unit, name)

    def get_context(self, beam, vehicle, cqi):
        context_str = "[" + str(beam.id) + "]" # arms are just beams, and
        return context_str                     # without vehicle context at all


####################################################################
## Scenario setup
####################################################################

class MyScenario(BaseScenario):
    '''This is MyScenario. It reimplements on_create() and on_event().'''

    def on_create(self, simworld) -> bool: # this will be called at the start

        ## simulation config
        self.time_scale = simworld.param["timescale"]
        self.sim_show_progress = 0.05 # show progress every x%
        self.sim_next_progress = self.sim_show_progress

        ## statistic collectors
        self.outcome = { "name": "Overall result" }
        self.outcome2 = { "name": "Sessional result" }

        ## load the map image and info
        self.map = MapInfo()
        self.map.load_file(image_file="paper-cmab.png",
                           data_file="paper-cmab.json")
        if not self.map.is_ready():
            print("Failed to load the map. The reason is: %s\n"%self.map.get_err_str())
            return False

        ## use the map and give a name to this scenario
        self.use_map(self.map)
        self.set_name("Multi-armed Bandit Beam-Vehicle Selection")

        ## load some pin location info from the map to place the BSs
        self.loc = { "BS": None }
        for pin_name in self.loc:
            self.loc[pin_name] = self.map.get_pin_xy(pin_name)
            if self.loc[pin_name]==None:
                print("Failed to load this pin: '%s'."%pin_name)
                return False

        ## create channel for signal propagation model
        freq = 28 # GHz
        ch_28GHz = Channel28GHz()
        #freq = 2.4 # GHz
        #beam_radius = self.map.km(0.3)

        ## create a BS with 6 beams
        self.bs_beams = []
        self.max_active_beam = 2
        beam_num = 6     # must be an integer
        beam_width = 360/beam_num
        beam_pointing = 0 # 0 means north

        for i in range(beam_num):
            facing_angle = beam_pointing + i*beam_width
            while facing_angle>=360: facing_angle-=360
            new_bs = MyBaseStationBeam(simworld, "BS-%02d"%(i+1),
                                       XY(xy=self.loc["BS"]), 
                                       freq, ch_28GHz, beam_width, facing_angle)
            self.bs_beams.append(new_bs)

        for beam in self.bs_beams:
            beam.show_coverage()

        ## create some vehicles
        self.all_vehicles = []
        self.loc = [ "East", "SouthWest", "South",
                     "CityCenter", "G-Live", "StationParking",
                     "London", "West", "Burpham", "North",
                     "HighStreet", "Station2", "Park" ]

        for i in range(100):
            this_vehicle = MyVehicle(simworld, "Car %d"%i, self.map, freq, ch_28GHz)
            while True:
                start = random.choice(self.loc)
                end = random.choice(self.loc)
                if end==start: continue
                if this_vehicle.set_route(start, end, speed_low=30, speed_high=50, 
                                          time_scale=self.time_scale):
                    break # route successful established, break
            self.all_vehicles.append(this_vehicle)

        ## set the algorithm to run
        if simworld.param["algo"]=="random":
            self.my_algorithm = RandomSelection(self)
        elif simworld.param["algo"]=="bestsnr":
            self.my_algorithm = HighestCQI(self) # or Best SNR
        elif simworld.param["algo"]=="mab":
            self.my_algorithm = MAB(self, self.time_scale)
        elif simworld.param["algo"]=="cmab":
            self.my_algorithm = CMAB(self, self.time_scale)
        else:
            print("Unrecognized algorithm name, random is used")
            self.my_algorithm = RandomSelection(self)

        if self.my_algorithm.get_name() in ["MAB","CMAB"]:
            self.my_algorithm.mab_setting(
                stop_time=simworld.param["mab exploration stop time"], # explore-first
                use_speed=simworld.param["mab speed"]
            )

        sim_sec = simworld.param["duration"]
        sim_min, sim_sec = divmod(sim_sec, 60)
        sim_hr,  sim_min = divmod(sim_min, 60)
        print(f"The algorithm to use is: {self.my_algorithm.get_name()}")
        print(f"Simulation duration = {sim_hr} hrs {sim_min} mins {sim_sec} secs")

        return True

    # this will be called repeatedly
    def on_event(self, sim_time, event_obj): 

        ## distribute the following events
        if event_obj==Event.MOBILITY_END: # a mobile node has finished its mobility?
            self.do_restart_node(sim_time,event_obj)
        elif event_obj==Event.SIM_MOBILITY: # mobility progresses a time step?
            self.do_mobility(sim_time,event_obj)
        elif event_obj==Event.SIM_END: # end of simulation?
            print("Sim done")
            self.show_outcome(1.0)

    ## end of mobility, create another path
    def do_restart_node(self, sim_time, event_obj):
        this_node = event_obj.get("node") # get the node reaching end of mobility
        while True:
            start = random.choice(self.loc)
            end = random.choice(self.loc)  # end location is a random choice
            if end==start: continue
            if this_node.set_route(start, end, speed_low=30, speed_high=50, time_scale=self.time_scale):
                break # route successful established, break

    ## Do user simulation here
    def do_mobility(self, sim_time, event_obj):

        ## iterate all beams to check its vehicle connectivity
        for beam in self.bs_beams:

            vehicle = beam.serving_node
            if vehicle==None: continue # skip if none

            (is_successful, _) = beam.comm.send_hello_to(vehicle)
            if not is_successful: # lost connection
                beam.lost_vehicle(sim_time)
                self.my_algorithm.report_conn_lost(sim_time,beam,vehicle)

        ## check for beam availability
        num_active_beam = 0
        for beam in self.bs_beams:
            if beam.serving_node!=None:
                num_active_beam += 1

        ## select vehicles to serve if needed
        while num_active_beam<self.max_active_beam:

            pair = self.my_algorithm.do_pairing(sim_time)
            if pair==None: break # no more available pairing, break the loop

            (beam,vehicle,cqi) = pair
            beam.connect_vehicle(vehicle,sim_time,cqi)
            self.my_algorithm.report_conn_confirmed(sim_time,beam,vehicle,cqi)
            num_active_beam += 1

        ## draw connectivity & bs coverage on the map
        for vehicle in self.all_vehicles:
            vehicle.show_connection()
        for beam in self.bs_beams:
            beam.show_coverage()

        ## show progress after progressing a certain percentage
        if sim_time/self.get_setting("stop time")>self.sim_next_progress:
            print("Sim progress = %2d%%"%(100*self.sim_next_progress))
            self.show_outcome(self.sim_next_progress)
            self.sim_next_progress += self.sim_show_progress

    def show_outcome(self, curr_progress):
        all_conn_info = []
        all_conn_info2 = []
        if curr_progress not in self.outcome: self.outcome[str(curr_progress)] = {}
        if curr_progress not in self.outcome2: self.outcome2[str(curr_progress)] = {}
        print("Beam, Overall Mean, Overall Count, Session Mean, Session Count")
        for beam in self.bs_beams:
            
            ## for oveall stats
            this_average = self.my_algorithm.stats[beam].get_average_connection_time()
            this_count = self.my_algorithm.stats[beam].get_connection_count()
            all_conn_info.append((this_average,this_count))
            output = "Beam '%s', %1.2f, %d, "%(beam.id, this_average, this_count)
            self.outcome[str(curr_progress)][beam.id] = (this_average,this_count)

            ## for this session only
            this_average = self.my_algorithm.stats2[beam].get_average_connection_time()
            this_count = self.my_algorithm.stats2[beam].get_connection_count()
            all_conn_info2.append((this_average,this_count))
            output += "%1.2f, %d"%(this_average, this_count)
            self.my_algorithm.stats2[beam].reset()
            self.outcome2[str(curr_progress)][beam.id] = (this_average,this_count)

            print(output)

        def get_final_average(conn_info):
            the_total = 0
            the_count = 0
            for conn in conn_info:
                (average,count) = conn
                the_count += count
                the_total += average*count
            if the_count==0: return 0
            return the_total/the_count
        print("Overall average = %1.2f"%get_final_average(all_conn_info))
        print("Average (this session)= %1.2f"%get_final_average(all_conn_info2))

        ## (#2846) for showing MAB & CMAB performances
        if False and self.my_algorithm.get_name() in ["MAB","CMAB"]:
            for context in sorted(self.my_algorithm.arm_total_reward):
                total_reward = self.my_algorithm.arm_total_reward[context]
                total_count = self.my_algorithm.arm_total_count[context]
                if total_count==0: continue # no completed record, skip
                mean_reward = total_reward/total_count
                print("%s: reward = %1.2f based on %d count(s)"%
                        (context, mean_reward, total_count))

        ## simulation completed?
        if curr_progress==1.0: 
            for outcome in [self.outcome,self.outcome2]:
                for progress in outcome:
                    if progress=="name": 
                        print("For %s:"%outcome["name"])
                        print(" progress percentage, BS-ID, mean, count,...")
                        continue
                    is_first_progress = True
                    for beam_id in outcome[progress]:
                        if is_first_progress:
                            print("%1.2f, "%float(progress),end="")
                            is_first_progress = False
                        (mean,count) = outcome[progress][beam_id]
                        print("%s, %1.2f, %d, "%(beam_id,mean,count), end="")
                    print("")
    

####################################################################
## The Main
####################################################################

if __name__ == "__main__":

    sim = World()

    ## user parameters
    sim.param = {}
    sim.param["duration"] = 10*3600    # seconds - standard run, 10 hours sim time
    #sim.param["duration"] = 3*3600    # seconds - short run, 3 hours sim time
    sim.param["timescale"] = 1.0       # one sim time unit = x secs (keep this to 1.0)
    sim.param["progress step"] = 0.05  # show progress every x%
    sim.param["animation"] = True   # comment this out to run without animatin
    #sim.param["random seed"] = 100  # comment this out to remove fixed random seed
    sim.param["mab exploration stop time"] = 0.3 * sim.param["duration"]
    sim.param["mab speed"] = False  # use vehicle speed as a feature in MAB?

    ## pick one algo to run
    #sim.param["algo"] = "random"
    #sim.param["algo"] = "bestsnr"
    #sim.param["algo"] = "mab"
    sim.param["algo"] = "cmab"

    ## setting control
    if "random seed" in sim.param:
        random.seed(sim.param["random seed"])
    animation_flag = False
    if "animation" in sim.param:
        animation_flag = sim.param["animation"]

    ## configure and run simulation
    sim.config(sim_stop=sim.param["duration"],
               sim_step=0.1, 
               sim_speed=1.0, 
               display_option=animation_flag,
               scenario=MyScenario(sim))
    sim.run()
