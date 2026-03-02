import numpy as np
# config.py
"""
Central place for all problem parameters.

Usage:
    from config import CFG
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple, Dict, Any


@dataclass(frozen=True)
class Config:
    ## ============== Parameters ===============================
    dt = 0.05
    T  = 10
    steps = int(T / dt)
    N = 5  # MPC horizon steps
    # Goal
    qg = np.array([4.0, -1])
    # Obstacles: (c, r)
    obstacles = [
        (np.array([1.5, 0]), 0.8),
        (np.array([3.0, -1.5]), 0.8)
    ]

    # CBF parameters
    KP = 0.2
    alpha = 0.5

    # MPC weights
    w_q = 2.0     # position-to-goal weight
    w_u   = 0.05    # control effort weight

    # Constraints
    u_max = 2.0     # acceleration bound (m/s^2)
    v_max = 2.5     # velocity bound

    # Initial condition
    q = np.array([0.0, 0.0])
    v = (qg - q)/np.linalg.norm(qg-q)
# Single global config instance (import this everywhere)
CFG = Config()


