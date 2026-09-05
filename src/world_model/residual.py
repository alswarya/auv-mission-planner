"""Residual MLP: learns the gap between analytical Fossen dynamics and the
simulator/real vehicle. s_{t+1} = s_t + d_physics(s_t, a_t) + d_residual(s_t, a_t)

Train on HoloOcean rollouts (scripts/collect_rollouts.py, scripts/train_residual.py).
Reference target: ~60% RMSE reduction over the analytical-only surrogate after
fine-tuning (arXiv:2608.19661).
"""
import torch
import torch.nn as nn


class ResidualMLP(nn.Module):
    def __init__(self, state_dim: int = 12, action_dim: int = 8,
                 hidden: tuple[int, ...] = (256, 256)):
        super().__init__()
        layers, din = [], state_dim + action_dim
        for h in hidden:
            layers += [nn.Linear(din, h), nn.ReLU()]
            din = h
        layers.append(nn.Linear(din, state_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat([state, action], dim=-1))
