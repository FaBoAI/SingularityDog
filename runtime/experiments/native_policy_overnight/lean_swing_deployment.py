import torch
from torch import nn
from .lean_swing_core import SwingCore

class DeployableSwingPolicy(nn.Module):
    def __init__(self,actor:nn.Module,use_plane:bool=True, stance_widen_m:float=.02, period:float=.56, residual_tau:float=0., heading_gain:float=.8, duty:float=.60, swing_height:float=.035, forward_command_limit:float=.36, hip_residual_scale:float=.27):
        super().__init__();self.actor=actor.cpu().eval();self.controller=SwingCore(1,'cpu',use_plane=use_plane,stance_widen_m=stance_widen_m,period=period,residual_tau=residual_tau,heading_gain=heading_gain,duty=duty,swing_height=swing_height,forward_command_limit=forward_command_limit,hip_residual_scale=hip_residual_scale)
        self.register_buffer('last_observation',torch.zeros(1,74));self.register_buffer('last_actor_output',torch.zeros(1,12))
    @torch.jit.export
    def reset(self,ids:torch.Tensor):
        self.controller.reset(ids);self.last_observation[ids]=0.;self.last_actor_output[ids]=0.
    def forward(self,gyro:torch.Tensor,gravity:torch.Tensor,command:torch.Tensor,q:torch.Tensor,dq:torch.Tensor,h:torch.Tensor)->torch.Tensor:
        obs=self.controller.observation(gyro,gravity,command,q,dq,h);self.last_observation.copy_(obs)
        raw=self.actor(obs);self.last_actor_output.copy_(raw)
        return self.controller.step_target(raw,command)
