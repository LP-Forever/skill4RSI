"""Site-specific identities, kept outside version control; importing never opens cameras."""
import json
import os
from pathlib import Path
CAMERA_NAMES = ('kinect_0', 'kinect_1', 'realsense_0', 'realsense_1')
path = Path(os.environ.get('SKILL4RSI_CAMERA_CONFIG', str(Path(__file__).with_name('cameras.local.json'))))
if path.is_file():
    CAMERAS = json.loads(path.read_text())
    if (not isinstance(CAMERAS, dict) or set(CAMERAS) != set(CAMERA_NAMES)
            or any(not isinstance(x, str) or not x.strip() for x in CAMERAS.values())
            or len(set(CAMERAS.values())) != 4):
        raise ValueError('Camera config requires four distinct nonempty identity strings')
    CAMERAS = {name: CAMERAS[name] for name in CAMERA_NAMES}
else:
    CAMERAS = {name: 'SET_' + name.upper() + '_IDENTITY' for name in CAMERA_NAMES}
