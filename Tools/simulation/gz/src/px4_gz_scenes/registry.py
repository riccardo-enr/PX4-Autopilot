"""
Environment registry — maps string names to scene factory functions.

Environment modules declare themselves with the ``@register_scene`` decorator.
Consumers look up environments by name via ``get_scene()``:

    from px4_gz_scenes import get_scene
    scene = get_scene("room")
    scene = get_scene("room", ext_x=20.0)

The registry is populated at import time when ``px4_gz_scenes.environments``
is first imported (which ``px4_gz_scenes/__init__.py`` does automatically).
"""

from __future__ import annotations

from typing import Callable

from px4_gz_scenes.scene import Scene

SceneFactory = Callable[..., Scene]

_REGISTRY: dict[str, SceneFactory] = {}


def register_scene(name: str) -> Callable[[SceneFactory], SceneFactory]:
    """Decorator that registers a scene factory under *name*.

    Args:
        name: The key used to retrieve this scene via ``get_scene()``.

    Raises:
        ValueError: if *name* is already registered.
    """

    def decorator(fn: SceneFactory) -> SceneFactory:
        if name in _REGISTRY:
            raise ValueError(f'Scene {name!r} is already registered. Each name must be unique.')
        _REGISTRY[name] = fn
        return fn

    return decorator


def get_scene(name: str, **kwargs) -> Scene:
    """Instantiate the scene registered under *name*.

    Args:
        name:   Registered scene name (e.g. ``"room"``).
        **kwargs: Forwarded to the factory function, allowing parameter
                  overrides such as ``ext_x=20.0``.

    Returns:
        A new :class:`Scene` instance.

    Raises:
        KeyError: if *name* is not in the registry.
    """
    if name not in _REGISTRY:
        available = ', '.join(sorted(_REGISTRY))
        raise KeyError(f'Unknown scene {name!r}. Available scenes: {available}')
    return _REGISTRY[name](**kwargs)


def list_scenes() -> list[str]:
    """Return a sorted list of all registered scene names."""
    return sorted(_REGISTRY)
