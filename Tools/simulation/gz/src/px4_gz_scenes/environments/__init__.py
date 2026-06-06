"""
Environment definitions — importing this package registers all built-in scenes.

Each sub-module uses the ``@register_scene`` decorator, which fires at import
time. This ``__init__`` imports every environment module so that the registry
is populated whenever ``px4_gz_scenes`` (or ``px4_gz_scenes.environments``)
is imported. No explicit registration call is needed by the consumer.
"""

from px4_gz_scenes.environments import office, room  # noqa: F401
