"""
Root conftest — suppress ROS2 pytest plugins that break on Python 3.13.

The launch_testing_ros entry point registers a hook
('pytest_launch_collect_makemodule') that is unknown to vanilla pytest,
causing a PluginValidationError during collection. We unregister it here
before pytest validates pending hooks.
"""


def pytest_configure(config) -> None:
    _BROKEN_PLUGINS = [
        'launch_ros',
        'launch_testing_ros_pytest_entrypoint',
        'ament_copyright',
        'ament_flake8',
        'ament_lint',
        'ament_pep257',
        'ament_xmllint',
    ]
    pm = config.pluginmanager
    for name in _BROKEN_PLUGINS:
        plugin = pm.get_plugin(name)
        if plugin is not None:
            pm.unregister(plugin)
