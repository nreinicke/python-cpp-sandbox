"""Module containing python classes and functions needed for powertrain modeling."""

import numpy as np
import pandas as pd
from pathlib import Path

def cumutrapz(x, y):
    z = np.zeros(len(x))
    z[0] = 0
    for i in np.arange(1, len(x)):
        z[i] = z[i-1] + 0.5 * (y[i] + y[i-1]) * (x[i] - x[i-1])
    return z

R_air = np.float64(287)  # J/(kg*K)

def get_rho_air(temperature_degC, elevation_m=180):
    """Returns air density [kg/m**3] for given elevation and temperature.
    Source: https://www.grc.nasa.gov/WWW/K-12/rocket/atmosmet.html
    Arguments:
    ----------
    temperature_degC : ambient temperature [°C]
    elevation_m : elevation above sea level [m].  
        Default 180 m is for Chicago, IL"""
    #     T = 15.04 - .00649 * h
    #     p = 101.29 * [(T + 273.1)/288.08]^5.256
    T_standard = 15.04 - 0.00649 * elevation_m  # nasa [degC]
    p = 101.29e3 * ((T_standard + 273.1) / 288.08) ** 5.256  # nasa [Pa]
    rho = p / (R_air * (temperature_degC + 273.15))  # [kg/m**3]

    return rho

class TimeTrace(object):
    "Class for storing time series of train speed and track properties."
    def __init__(self, tt_dict:dict=None, tt_file:Path=None):
        """
        Arguments:
        ----------
        tt_dict: dict, dictionary containing values for 
            `time_s`, `speed`, `grade`, and `curvature`
        tt_file: Path or str, path to file containing time trace information.
        
        Pass either tt_dict or tt_file but not both.
        """
        if tt_dict and tt_file:
            print('`tt_dict` is overriding `tt_file` since both are provided')
        
        if not(tt_dict):
            assert tt_file
            tt_file = Path(tt_file) # make sure it's path to be OS-independent
            tt_dict = pd.read_csv(tt_file).to_dict()
        
        self.time_s = tt_dict.pop('time_s')
        # variables preceded by `_` are not to be modified
        # baseline speed
        self._speed_m__s = tt_dict.pop('speed_m__s')
        # actual speed
        self.speed_m__s = self._speed_m__s
        # baseline distance for interpolating distance-dependent properties
        self._dist_m = cumutrapz(self.time_s, self._speed_m__s)
        # baseline for distance-dependent properties
        self._grade = tt_dict.pop('grade', np.zeros(len(self.time_s))) # grade [rise/run]
        self._curvature_m = tt_dict.pop('curvature_m', np.zeros(len(self.time_s))) # radius of curvature

        assert len(tt_dict) == 0, f"Unepected columns in `tt_dict`: {tt_dict.keys()}"

    @property
    def dt_s(self):
        return np.append(np.zeros(1), self.time_s[1:] - self.time_s[:-1])
    
    # the following properties insure that spatial parameters are correctly preserved
    @property
    def dist_m(self):
        return cumutrapz(self.time_s, self.speed_m__s)
    @property
    def grade(self):
        return np.interp(self.dist_m, self._dist_m, self._grade)
    @property
    def curvature_m(self):
        return np.interp(self.dist_m, self._dist_m, self._curvature_m)


class BasicTrain(object):
    """
    Class for energy-absorbing dynamics of train -- e.g. inertia, rolling resistance, drag, grade, curvature.
    """
    def __init__(self, train_dict=None, train_file=None,):
        """
        Arguments:
        ----------
        train_dict: dict, dictionary containing values for train model
        train_file: Path or str, path to file containing time trace information.
        
        Pass either train_dict or train_file but not both.
        """

        if train_dict and train_file:
            print('`train_dict` is overriding `train_file` since both are provided')
        
        if not(train_dict):
            assert train_file
            train_file = Path(train_file) # make sure it's path to be OS-independent
            train_dict = pd.read_csv(train_file).to_dict()
        
        self.roll_resist = train_dict.pop('roll_resist')
        self.drag_coeff = train_dict.pop('drag_coeff')
        self.frnt_area_m2 = train_dict.pop('frnt_area_m2')
        self.m_kg = train_dict.pop('m_kg')

        assert len(train_dict) == 0, f"Unepected columns in `tt_dict`: {train_dict.keys()}"

class LocomotiveConsist(object):
    """
    Class for powertrain dynamics of consist -- e.g. controls, engine, battery, e-machines
    """
    def __init__(self, loco_dict=None, loco_file=None,):
        """
        Arguments:
        ----------
        loco_dict: dict, dictionary containing values for train model
        loco_file: Path or str, path to file containing time trace information.
        
        Pass either loco_dict or loco_file but not both.
        """

        if loco_dict and loco_file:
            print('`loco_dict` is overriding `loco_file` since both are provided')
        
        if not(loco_dict):
            assert loco_file
            loco_file = Path(loco_file) # make sure it's path to be OS-independent
            loco_dict = pd.read_csv(loco_file).to_dict()

        # fuel converter (e.g. engine, fuel cell) peak power
        self.fc_pw_peak_W = loco_dict.pop('fc_pw_peak_W')
        # fuel converter efficiency array
        self.fc_eta_array = loco_dict.pop('fc_eta_array')      
        # fuel converter power fraction array at which efficiencies are evaluated
        self.fc_pwr_frac_array = loco_dict.pop('fc_pwr_frac_array') 

        assert len(loco_dict) == 0     

      
POSSIBLE_TRAIN_SOLVERS = ["Python", "C++"]
PYTHON_TRAIN_SOLVER = POSSIBLE_TRAIN_SOLVERS[0]

accel_grav_m__s = 9.81 # acceleration due to gravity

class TrainSimulation(object):
    """
    Class for running locomotive consist and powertrain models.  
    TODO Break this up into separate objects for train dynamics and powertrain dynamics to show 
    """
    def __init__(self, loco_con:LocomotiveConsist, train:BasicTrain, time_trace:TimeTrace, train_solver=PYTHON_TRAIN_SOLVER):
        self.loco_con = loco_con
        self.train = train
        self.time_trace = time_trace
        self.train_solver = train_solver
        self.init_time_arrays()

    def init_time_arrays(self):
        "Initialize arrays to be updated at each time step."
        # achieved speed
        self.rr_pwr_W = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.drag_pwr_W = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.accel_pwr_W = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.brake_pwr_W = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.rho_air_kg__m3 = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.req_pwr_W = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.fuel_pwr_W = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.trace_miss_iters = np.zeros(len(self.time_trace.time_s), dtype=np.float64)
        self.trace_met = np.array([False] * len(self.time_trace.time_s), dtype=np.bool_)
        
    def walk(self):
        for i in range(1, len(self.time_trace.time_s)):
            self.step(i)
    
    def step(self, i):
        self.solve_step(i)
        
    def solve_step(self, i):
        self.solve_required_power(i)
        self.solve_missed_trace(i)
        self.solve_braking(i)
        self.solve_energy_consumption(i)

    def solve_required_power(self, i):
        """
        Sets power requirements based on:
        - rolling resistance
        - drag
        - inertia
        - grade
        - rail curvature
        - acceleration
        - braking
        (some of these aren't implemented yet)
        """
        if self.train_solver == PYTHON_TRAIN_SOLVER:
            self.rho_air_kg__m3[i] = get_rho_air(22.0, ) # TODO actually feed in variables
            self.rr_pwr_W[i] = self.train.roll_resist * np.cos(np.arctan(self.time_trace.grade[i])) * self.train.m_kg * accel_grav_m__s * \
                0.5 * (self.time_trace.speed_m__s[i-1] + self.time_trace.speed_m__s[i]) 
            self.drag_pwr_W[i] = 0.5 * self.rho_air_kg__m3[i] * self.train.drag_coeff * self.train.frnt_area_m2 * (
                0.5 * (self.time_trace.speed_m__s[i-1] + self.time_trace.speed_m__s[i])) ** 3
            self.accel_pwr_W[i] = self.train.m_kg / (2 * self.time_trace.dt_s[i]) * (self.time_trace.speed_m__s[i] ** 2 - self.time_trace.speed_m__s[i-1] ** 2)
            # TODO add grade power
            # TODO add curvature power
            self.req_pwr_W[i] = self.rr_pwr_W[i] + self.drag_pwr_W[i] + self.accel_pwr_W[i]
        else:
            pass # TODO implement API for C++: https://github.nrel.gov/AVCI/altrios_nrel/issues/1

    def solve_missed_trace(self, i):
        "Finds speed at which powertrain can produce necessary power, if specified speed exceeds capabilities."
        # TODO change the following to account for Hybrid Electric Consist (HEC) 
        # TODO implement time dilation
        self.trace_met[i] = self.req_pwr_W[i] <= self.loco_con.fc_pw_peak_W
        
        if not(self.trace_met[i]):
            # figure out the speed that can actually be achieved
            # guess based on speed scaling with power ** 1/3
            speed_guesses = self.time_trace.speed_m__s[i] * np.array([1, (self.loco_con.fc_pw_peak_W / self.req_pwr_W[i]) ** 0.333])
            pwr_short = np.array([self.req_pwr_W[i] - self.loco_con.fc_pw_peak_W])
        while not(self.trace_met[i]):
            # Newton's method for finding speed at which power requiremnts can be satisfied
            self.trace_miss_iters[i] += 1 # increment iteration counter
            self.time_trace.speed_m__s[i] = speed_guesses[-1]
            self.solve_required_power(i)
            pwr_short = np.append(pwr_short, self.req_pwr_W[i] - self.loco_con.fc_pw_peak_W)
            speed_guesses = np.append(
                speed_guesses, 
                speed_guesses[-1] - (speed_guesses[-1] - speed_guesses[-2]) / (pwr_short[-1] - pwr_short[-2])
            )
            self.time_trace.speed_m__s[i] = speed_guesses[-1]
            self.trace_met[i] = self.req_pwr_W[i] <= self.loco_con.fc_pw_peak_W
    
    def solve_braking(self, i):
        "Determinse brake power requirements and solves for regen braking."
        self.brake_pwr_W[i] = max(0, -self.req_pwr_W[i])
        # TODO figure out what regen brake power should be

    def solve_energy_consumption(self, i):
        "Solves for battery and fuel power consumption."
        if self.req_pwr_W[i] > 0:
            self.fuel_pwr_W[i] = self.req_pwr_W[i] / np.interp(
                self.req_pwr_W[i], 
                self.loco_con.fc_pwr_frac_array * self.loco_con.fc_pw_peak_W, 
                self.loco_con.fc_eta_array)


def run(speed_trace_m__s, time_s):
    time_trace = TimeTrace(tt_dict={
        "time_s":np.array(time_s),
        "speed_m__s":np.array(speed_trace_m__s),
    })

    train = BasicTrain(train_dict={
        "roll_resist": 0.005,
        "drag_coeff": 10,
        "frnt_area_m2": 5,
        "m_kg": 50e3,
    })

    loco_con = LocomotiveConsist(loco_dict={
        "fc_pw_peak_W": 500e3, 
        "fc_pwr_frac_array": np.linspace(0.1, 1, 5),
        "fc_eta_array": np.array([0.2, 0.32, 0.35, 0.4, 0.38]),
    })


    train_sim = TrainSimulation(loco_con=loco_con, train=train, time_trace=time_trace)
    train_sim.walk()

    return 1 