import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie import syncCrazyflie
from cflib.crazyflie.log import LogConfig
# https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/swarm/swarmSequence.py

def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


rotation = {} # Dictionaty of URI and rotation

def stablizer_callback(uri, timestamp, data, logconf):
    roll = data['stateEstimate.roll']
    pitch = data['stateEstimate.pitch']
    yaw = data['stateEstimate.yaw']
    stabilizer = (roll, pitch, yaw)

    rotation[uri] = stabilizer
    return(stabilizer)

def GetStabilizer(scf):
    log_conf = LogConfig(name='stateEstimateData', period_in_ms=100)
    log_conf.add_variable('stateEstimate.roll', 'float')
    log_conf.add_variable('stateEstimate.pitch', 'float')
    log_conf.add_variable('stateEstimate.yaw', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: stablizer_callback(scf.cf.link_uri, timestamp, data, logconf))
    log_conf.start()

def propeller_check(scf):
    print("sendddd")
    # scf.cf.commander.send_hover_setpoint(vx=1, vy=0, yawrate=0, zdistance=0.4)
    scf.cf.commander.send_setpoint(roll=10, pitch=0, yawrate=10, thrust=int(20000))
    # scf.cf.commander.send_position_setpoint(0,0,2,0)
    # scf.cf.commander.send_velocity_world_setpoint(0,0,2,0)
    time.sleep(0.1)

uris = {
    'radio://0/80/2M/E7E7E7E7E6',
    'radio://0/80/2M/E7E7E7E7E8',
    'radio://0/110/2M/E7E7E7E7E9'
    # Add more URIs if you want more copters in the swarm
}

if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    # cflib.crtp.init_drivers()
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.openlinks()
        swarm.parallel_safe(wait_for_param_download)
        swarm.sequential(add_callback_func)
        cnt = 0
        while cnt <10:
            swarm.sequential(propeller_check)
            cnt=cnt+1
            # print(rotation)
    print('finishhhh')
