"""Core interfaces and state abstractions for NRDAS AMR Fleet Coordination."""

from .fleet_state import FleetState, RobotInfo
from .interfaces import (
    DeadlockManager,
    GlobalPlanner,
    LocalPlanner,
    MetricsCollector,
    TaskAllocator,
)
from .state_machine import (
    InvalidStateTransitionError,
    RobotLifecycleState,
    RobotStateMachine,
)

__all__ = [
    'TaskAllocator',
    'GlobalPlanner',
    'LocalPlanner',
    'DeadlockManager',
    'MetricsCollector',
    'RobotLifecycleState',
    'RobotStateMachine',
    'InvalidStateTransitionError',
    'FleetState',
    'RobotInfo',
]
