"""Create a loopback-only ROS context without changing contestant child environments."""
import os
import threading
from pathlib import Path
from rclpy.utilities import get_rmw_implementation_identifier

from rclpy.context import Context

_ENV_LOCK = threading.Lock()


def internal_domain():
    public = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    private = int(os.environ.get('RMTC_INTERNAL_DOMAIN_ID', str((public + 1) % 233)))
    if not 0 <= public <= 232 or not 0 <= private <= 232 or public == private:
        raise ValueError('public/internal ROS domains must differ and be in 0..232')
    return private


def internal_context():
    if get_rmw_implementation_identifier() != 'rmw_fastrtps_cpp':
        raise ValueError('internal isolation requires RMW_IMPLEMENTATION=rmw_fastrtps_cpp')
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_DISCOVERY_SERVER'):
        if os.environ.get(key):
            raise ValueError('internal isolation does not allow transport override: ' + key)
    if any(Path(name).exists() for name in ('DEFAULT_FASTRTPS_PROFILES.xml', 'DEFAULT_FASTDDS_PROFILES.xml')):
        raise ValueError('remove working-directory DDS profiles before starting isolated organizer')
    context = Context()
    # rclpy Humble exposes domain_id but reads localhost_only from the environment
    # during context initialization. Restore it before spawning any public nodes.
    with _ENV_LOCK:
        previous = os.environ.get('ROS_LOCALHOST_ONLY')
        try:
            os.environ['ROS_LOCALHOST_ONLY'] = '1'
            context.init(args=[], domain_id=internal_domain(), initialize_logging=False)
        finally:
            if previous is None:
                os.environ.pop('ROS_LOCALHOST_ONLY', None)
            else:
                os.environ['ROS_LOCALHOST_ONLY'] = previous
    return context
