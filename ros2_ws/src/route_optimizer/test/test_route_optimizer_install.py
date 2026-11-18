from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import route_optimizer


def test_installed_package():
    share = Path(get_package_share_directory(route_optimizer.__name__))
    assert (share / 'package.xml').is_file()
