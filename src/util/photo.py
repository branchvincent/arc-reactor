
from past.builtins import basestring

def location_camera2photo_url(location, camera):
    return '/photos/{}/{}/'.format(location, camera)

def url2location_camera(url):
    if isinstance(url, basestring):
        url = [x for x in url.split('/') if x]

    return url[1:3]
