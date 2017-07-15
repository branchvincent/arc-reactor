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

if __name__ == '__main__':
    init()
