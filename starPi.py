#!/usr/bin/env python3
"""
Script base RocketPy con airbrakes controllati da una funzione custom.

Step futuri:
- sostituire la logica di controllo in `airbrake_controller` con un PID
- usare AI/evolutionary per ottimizzare Kp, Ki, Kd del PID
"""
import random as pyrandom
from rocketpy import Environment, SolidMotor, Rocket, Flight
import matplotlib.pyplot as plt
import os
from pid import PID 

def noisy_measurements(altitude_true, vz_true, dt):
    # baro noise
    sigma_baro = 1.5  # m
    baro_noise = pyrandom.gauss(0, sigma_baro)

    # baro drift (0.1 m/min)
    drift_rate = 0.002  # m/s drift
    drift = drift_rate * dt
    if not hasattr(noisy_measurements, "baro_drift"):
        noisy_measurements.baro_drift = 0
    noisy_measurements.baro_drift += pyrandom.gauss(0, drift)

    altitude_meas = altitude_true + baro_noise + noisy_measurements.baro_drift

    # velocity noise (IMU integration)
    sigma_accel = 0.03 * 9.81
    vz_noise = pyrandom.gauss(0, sigma_accel * dt)

    vz_meas = vz_true + vz_noise

    return altitude_meas, vz_meas


# global variables for Pid control
ENV = None
MOTOR = None
PID_CTRL = None  # verrà creato in main/simulate
TARGET_APOGEE_AGL = 3000.0  # apogeo target in metri (puoi cambiarlo)

# -------------------------
# CONFIGURAZIONE FILE DATI
# -------------------------

# Percorso base dei file dati (adatta alla tua struttura)
DATA_DIR = "./data"

MOTOR_FILE = os.path.join(
    DATA_DIR, "motors", "cesaroni", "Cesaroni_M1670.eng"
)
POWER_OFF_DRAG_FILE = os.path.join(
    DATA_DIR, "rockets", "calisto", "powerOffDragCurve.csv"
)
POWER_ON_DRAG_FILE = os.path.join(
    DATA_DIR, "rockets", "calisto", "powerOnDragCurve.csv"
)
AIRBRAKES_CD_FILE = os.path.join(
    DATA_DIR, "rockets", "calisto", "air_brakes_cd.csv"
)


# -------------------------
# COSTRUZIONE AMBIENTE
# -------------------------
def build_environment():
    """
    Crea l'ambiente RocketPy.
    Puoi cambiare lat/long/elevation secondo il sito EuroC o eventuale campo test.
    """
    env = Environment( #lat long of Euroc launch site
        latitude=39.3897,
        longitude=-8.28897,
        elevation=160,  # m
    )

    # data and atmospheric model
    env.set_date((2025, 10, 15, 12))  # 2025-10-15 12:00 UTC
    env.set_atmospheric_model(type="standard_atmosphere")

    #se vogliamo aggiungere vento fisso
    #ENV.set_atmospheric_model(
    #type="custom_atmosphere",
    #wind_u=lambda h: -4,   # componente verso ovest/est
    #wind_v=lambda h: 2,    # componente verso nord/sud
    #)

    return env


# -------------------------
# COSTRUZIONE MOTORE + RAZZO
# -------------------------
def build_rocket(env):
    """
    Crea motore e razzo "tipo Calisto" come negli esempi ufficiali RocketPy.
    Adatta in seguito a geometrie/massa del vostro razzo EuroC.
    """

    motor = SolidMotor(
        thrust_source=MOTOR_FILE,
        dry_mass=1.815,                        # kg
        dry_inertia=(0.125, 0.125, 0.002),     # kg*m^2
        nozzle_radius=33 / 1000,
        grain_number=5,
        grain_density=1815,
        grain_outer_radius=33 / 1000,
        grain_initial_inner_radius=15 / 1000,
        grain_initial_height=120 / 1000,
        grain_separation=5 / 1000,
        grains_center_of_mass_position=0.397,
        center_of_dry_mass_position=0.317,
        nozzle_position=0,
        burn_time=3.9,
        throat_radius=11 / 1000,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )

    rocket = Rocket(
        radius=127 / 2000,
        mass=14.426,  # massa senza motore
        inertia=(6.321, 6.321, 0.034),
        power_off_drag=POWER_OFF_DRAG_FILE,
        power_on_drag=POWER_ON_DRAG_FILE,
        center_of_mass_without_motor=0,
        coordinate_system_orientation="tail_to_nose",
    )

    # Rail buttons
    rocket.set_rail_buttons(
        upper_button_position=0.0818,
        lower_button_position=-0.618,
        angular_position=45,
    )

    # Aggiungi motore
    rocket.add_motor(motor, position=-1.255)

    # Nosecone
    rocket.add_nose(
        length=0.55829,
        kind="vonKarman",
        position=1.278,
    )

    # Fins
    rocket.add_trapezoidal_fins(
        n=4,
        root_chord=0.120,
        tip_chord=0.060,
        span=0.110,
        position=-1.04956,
        cant_angle=0.5,
        airfoil=(os.path.join(DATA_DIR, "airfoils", "NACA0012-radians.txt"), "radians"),
    )

    # Tail
    rocket.add_tail(
        top_radius=0.0635,
        bottom_radius=0.0435,
        length=0.060,
        position=-1.194656,
    )

    return rocket, motor


# -------------------------
# CONTROLLER AIRBRAKES
# -------------------------
def airbrake_controller(time, sampling_rate, state, state_history, observed_variables, air_brakes):
    global ENV, MOTOR, PID_CTRL, TARGET_APOGEE_AGL

    altitude_ASL = state[2]
    vx, vy, vz = state[3], state[4], state[5]
    dt = 1.0 / sampling_rate

    # noisy measurements
    altitude_meas, vz_meas = noisy_measurements(
        altitude_ASL - ENV.elevation,
        state[5],
        dt
    )
    altitude_AGL = altitude_meas
    vz = vz_meas


    # Fase di boost: niente airbrake ma logghiamo comunque
    if time < MOTOR.burn_out_time:
        air_brakes.deployment_level = 0.0
    else:
        # -----------------------
        # DEFINIZIONE DEL SETPOINT DI VELOCITÀ
        # -----------------------
        # distanza mancante all'apogeo target (non meno di 0)
        remaining = max(TARGET_APOGEE_AGL - altitude_AGL, 0.0)

        # coefficiente per trasformare distanza in velocità di riferimento
        k_v = 0.2  # da tarare / ottimizzare
        v_ref = k_v * remaining  # più sei lontano, più puoi salire veloce

        # clip della v_ref per evitare numeri enormi
        v_ref = min(v_ref, 200.0)  # m/s limite alto

        # errore per il PID
        error = v_ref - vz

        # aggiornamento PID → output = deployment_level
        u = PID_CTRL.update(error)
        # noise servo
        servo_noise = pyrandom.gauss(0, 0.02)

        # deadband
        if abs(u) < 0.01:
            u = 0.0

        # rate limit
        if not hasattr(air_brakes, "prev_u"):
            air_brakes.prev_u = 0.0

        max_rate = 0.25  # per secondo
        u_rate_limited = max(
            min(u, air_brakes.prev_u + max_rate * dt),
            air_brakes.prev_u - max_rate * dt
        )

        air_brakes.prev_u = u_rate_limited + servo_noise
        air_brakes.deployment_level = air_brakes.prev_u


    # --- calcolo mach e Cd per log ---
    wind_x = ENV.wind_velocity_x(altitude_ASL)
    wind_y = ENV.wind_velocity_y(altitude_ASL)
    free_stream_speed = ((wind_x - vx) ** 2 + (wind_y - vy) ** 2 + vz ** 2) ** 0.5
    mach = free_stream_speed / ENV.speed_of_sound(altitude_ASL)
    cd_brake = air_brakes.drag_coefficient(air_brakes.deployment_level, mach)

    return (
        time,
        air_brakes.deployment_level,
        cd_brake,
        altitude_AGL,
        vz,
    )



# -------------------------
# MAIN: Simulation e plotting
# -------------------------
def simulate(Kp, Ki, Kd, make_plots=False):
    global ENV, MOTOR, PID_CTRL

    ENV = build_environment()
    rocket, MOTOR = build_rocket(ENV)

    sampling_rate = 10
    dt = 1.0 / sampling_rate
    PID_CTRL = PID(Kp, Ki, Kd, dt, u_min=0.0, u_max=1.0)
    PID_CTRL.reset()

    air_brakes = rocket.add_air_brakes(
        drag_coefficient_curve=AIRBRAKES_CD_FILE,
        controller_function=airbrake_controller,
        sampling_rate=sampling_rate,
        reference_area=None,
        clamp=True,
        initial_observed_variables=[0.0, 0.0, 0.0, 0.0, 0.0],
        override_rocket_drag=False,
        name="AirBrakesDemo",
    )

    flight = Flight(
        rocket=rocket,
        environment=ENV,
        rail_length=5.2,
        inclination=84,
        heading=133,
        time_overshoot=False,
        terminate_on_apogee=True,
    )

    # --- ottieni apogeo AGL ---
    apogee_ASL = flight.apogee
    apogee_AGL = apogee_ASL - ENV.elevation

    # --- carica log del controller ---
    obs = flight.get_controller_observed_variables()
    t_list, level_list, h_agl_list = [], [], []
    for t, level, cd, h_agl, vz in obs:
        t_list.append(t)
        level_list.append(level)
        h_agl_list.append(h_agl)

    # energia / fatica attuatore ~ somma delle variazioni di deployment
    actuator_effort = 0.0
    for i in range(1, len(level_list)):
        actuator_effort += abs(level_list[i] - level_list[i-1])

    # errore apogeo
    apogee_error = abs(TARGET_APOGEE_AGL - apogee_AGL)

    # funzione costo (da minimizzare)
    w1, w2 = 1.0, 10.0    # pesa apogeo vs fatica
    cost = w1 * apogee_error + w2 * actuator_effort

    # plot opzionali
    if make_plots:
        print(f"Apogee AGL = {apogee_AGL:.1f} m, error = {apogee_error:.1f} m, actuator_effort = {actuator_effort:.3f}, cost = {cost:.1f}")
        plt.figure(figsize=(10,4))
        plt.subplot(1,2,1)
        plt.plot(t_list, h_agl_list)
        plt.xlabel("Time [s]")
        plt.ylabel("Altitude AGL [m]")
        plt.title("Altitude AGL vs time (controller steps)")
        plt.grid(True)

        plt.subplot(1,2,2)
        plt.plot(t_list, level_list)
        plt.xlabel("Time [s]")
        plt.ylabel("Airbrake deployment level")
        plt.title("Airbrake deployment vs time")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    return cost, apogee_AGL, actuator_effort

if __name__ == "__main__":
    cost, apogee, effort = simulate(0.0005, 0.00001, 0.0003, make_plots=True)
    print("Cost:", cost, "Apogee:", apogee, "Effort:", effort)
