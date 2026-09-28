"""Opt-in actor fusion candidate; deployed loaders never import this module."""
import torch
from torch import Tensor

from ..lean_swing_deployment import DeployableSwingPolicy


class FusedActorPolicy(DeployableSwingPolicy):
    def forward(self, gyro: Tensor, gravity: Tensor, command: Tensor,
                q: Tensor, dq: Tensor, h: Tensor) -> Tensor:
        obs = self.controller.observation(gyro, gravity, command, q, dq, h)
        self.last_observation.copy_(obs)
        raw = torch.ops.sd_actor_fileonly_r1.forward(
            obs, self.actor[0].weight, self.actor[0].bias,
            self.actor[2].weight, self.actor[2].bias,
            self.actor[4].weight, self.actor[4].bias,
            self.actor[6].weight, self.actor[6].bias)
        self.last_actor_output.copy_(raw)
        return self.controller.step_target(raw, command)
