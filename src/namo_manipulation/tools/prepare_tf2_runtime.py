#!/usr/bin/env python3
"""Extract two official TF fixes locally; never install or update system ROS."""
import hashlib
import json
from pathlib import Path
import subprocess

PACKAGES = {
    'ros-humble-tf2': ('0.25.24-1jammy.20260915.203746',
        'f6b5d1d098a9ba9080e2c24cccf98a669f1f8bb878602b5b53bd55a4cfbd0cfc'),
    'ros-humble-tf2-ros': ('0.25.24-1jammy.20260915.204410',
        'cc7ff31644124f5e601576996e078fe2c6d51c6dbf730182d44db4f75651ed08'),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    workspace = Path(__file__).resolve().parents[2]
    base = workspace/'.namo_manip/runtime_fix'
    debs = base/'debs'; debs.mkdir(parents=True, exist_ok=True)
    report = {'source': 'official packages.ros.org ROS Humble Jammy amd64 binaries',
              'purpose': 'geometry2 #990: fix waitForTransform/testTransformableRequests ABBA deadlock',
              'system_modified': False, 'packages': {}, 'libraries': {}}
    for name, (version, checksum) in PACKAGES.items():
        archive = debs/f'{name}_{version}_amd64.deb'
        if not archive.is_file():
            subprocess.run(['apt-get', 'download', f'{name}={version}'], cwd=debs, check=True)
        if sha(archive) != checksum:
            raise RuntimeError(f'SHA256 mismatch: {archive}; refusing extraction')
        subprocess.run(['dpkg-deb', '-x', str(archive), str(base/'root')], check=True)
        report['packages'][name] = {'version': version, 'deb_sha256': checksum}
    for name in ('libtf2.so', 'libtf2_ros.so'):
        report['libraries'][name] = sha(base/'root/opt/ros/humble/lib'/name)
    (base/'manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
