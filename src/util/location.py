
def location_base_url(location):
    if location.startswith('bin'):
        reference_url = '/shelf/bin/{}/'.format(location)
    elif location.endswith('tote'):
        reference_url = '/tote/{}/'.format(location[:-5])
    elif location.startswith('box'):
        reference_url = '/box/{}/'.format(location)
    elif location == 'inspect':
        reference_url = 'robot/inspect_'
    else:
        raise RuntimeError('unrecognized item location: {}'.format(location))

    return reference_url

def location_pose_url(location):
    return location_base_url(location) + 'pose'

def location_bounds_url(location):
    return location_base_url(location) + 'bounds'
