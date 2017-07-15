from pensive.client import PensiveClient
from master.world import build_world

def init(store=None):
    store = store or PensiveClient().default()
    world = build_world(store)
    robot = world.robot('tx90l')
    qmid = []

    for ql,qu in zip(*robot.getJointLimits()):
        qi = (ql+qu)/2
        qmid.append(qi)
    store.put('robot/current_config', qmid)

def updateMotionLimits(store=None):
    def _getValsBelowCeiling(vals, vals_ceiling):
        vals_new = []
        for v, v_ceil in zip(vals, vals_ceiling):
            vals_new.append(v if v < v_ceil else v_ceil)
        return vals_new

    store = store or PensiveClient().default()
    world = build_world(store)
    robot = world.robot('tx90l')
    vmax_abs = robot.getVelocityLimits()
    amax_abs = robot.getAccelerationLimits()

    # Ensures all v/a limits respect the limits in the rob file
    for state, values in store.get('planner/states').items():
        vmax = values['joint_velocity_limits']
        amax = values['joint_acceleration_limits']
        vmax_new = _getValsBelowCeiling(vmax, vmax_abs)
        amax_new = _getValsBelowCeiling(amax, amax_abs)
        store.put(['planner', 'states', state, 'joint_velocity_limits'], vmax_new)
        store.put(['planner', 'states', state, 'joint_acceleration_limits'], amax_new)


if __name__ == '__main__':
    init()
    updateMotionLimits()
