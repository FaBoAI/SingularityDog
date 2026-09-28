"""L13 registered hip residual mapping with unchanged L11 sensor feedback.

L13 contracts each leg residual toward feasible reference q, then searches an
IMU-up projection with verified IK and 0.02 rad joint margins. Unmet requests and
constraint corrections are explicit; physical clipping is a separate diagnostic.
Legacy raw-clamp/previous_clipped semantics and 74 actor fields stay fixed.

The legacy 69 observation fields and vx/vy/active filter layout are preserved.
Append yaw filter stages 0/1/2, sin(relative heading error), cos(error): 74 fields.
Observation caches IMU/joint samples but never integrates controller time/state.
Step integrates the IMU-derived Z-Y-X heading rate once at 50Hz. No ground-truth pose,
translation velocity, contact or height is supplied to the actor or reference.
All integration/IK remains float64 with the selected public output dtype.
"""
from typing import Dict
import torch
from torch import Tensor, nn

class SwingCore(nn.Module):
    __constants__ = ['num_envs','period','duty','filter_tau','bootstrap_s','dt','observation_dim','action_dim','use_plane','swing_height','max_projection','stance_widen_m','residual_alpha','heading_gain','heading_correction_limit','forward_command_limit','hip_residual_scale','joint_margin','projection_iterations','projection_scan_intervals']
    def __init__(self, num_envs: int, device: str='cpu', dtype: torch.dtype=torch.float32, use_plane: bool=True, stance_widen_m:float=.02, period:float=.56, residual_tau:float=0., heading_gain:float=.8, duty:float=.60, swing_height:float=.035, forward_command_limit:float=.36, hip_residual_scale:float=.27):
        super().__init__()
        if num_envs<1:raise ValueError('num_envs must be positive')
        if period not in (.56,.50) or duty not in (.60,.55) or swing_height not in (.035,.030) or forward_command_limit not in (.36,.40,.44,.46):raise ValueError('Unregistered L13 reference parameters')
        if residual_tau!=0.:raise ValueError('L11 retains zero residual filtering tau')
        if hip_residual_scale not in (.30,.27):raise ValueError('Registered L13 hip residual scales are 0.30 and 0.27 rad')
        self.hip_residual_scale=hip_residual_scale
        self.num_envs=num_envs;self.period=period;self.duty=duty;self.filter_tau=.15;self.bootstrap_s=2.;self.dt=.02
        self.forward_command_limit=forward_command_limit
        self.residual_alpha=0.
        if heading_gain not in (0.,.8):raise ValueError('Registered heading gains are 0 and 0.8 /s')
        self.heading_gain=heading_gain;self.heading_correction_limit=.12
        self.observation_dim=74;self.action_dim=12
        opts={'dtype':torch.float64,'device':device}
        self.register_buffer('output_anchor',torch.empty(0,dtype=dtype,device=device))
        self.register_buffer('phase',torch.zeros(num_envs,**opts))
        self.register_buffer('elapsed',torch.zeros(num_envs,**opts))
        self.register_buffer('filters',torch.zeros((num_envs,3,3),**opts))
        self.register_buffer('yaw_filters',torch.zeros((num_envs,3),**opts))
        self.register_buffer('heading_error_rad',torch.zeros(num_envs,**opts))
        self.register_buffer('sensor_yaw_rate',torch.zeros(num_envs,**opts))
        self.register_buffer('previous_clipped',torch.zeros((num_envs,12),**opts))
        self.register_buffer('origins',torch.tensor([[.155,.110,0],[.155,-.110,0],[-.155,.110,0],[-.155,-.110,0]],**opts))
        self.register_buffer('anchor',torch.tensor([[.155,.10999999999999999,-.23156651958078514],[.155,-.10999999999999999,-.23156651958078514],[-.155,.10999999999999999,-.23156651958078514],[-.155,-.10999999999999999,-.23156651958078514]],**opts))
        self.register_buffer('offsets',torch.tensor([0.,.5,.5,0.],**opts))
        self.register_buffer('signs',torch.tensor([1.,-1.,1.,-1.],**opts))
        q0=[-.2800237445733691,.3837207788252659,-.7674415576505315,.2800237445733691,.3837207788252659,-.7674415576505315]*2
        self.register_buffer('nominal',torch.tensor(q0,**opts))
        self.register_buffer('lower',torch.tensor([-.5,-.9,-2.2]*4,**opts))
        self.register_buffer('upper',torch.tensor([.5,1.2,-.08]*4,**opts))
        self.register_buffer('scale',torch.tensor([hip_residual_scale,.40,.50]*4,**opts))
        self.stance_widen_m=stance_widen_m
        if stance_widen_m not in (0.,.02,.03):raise ValueError("Finite preregistered stance widths only")
        self.use_plane=use_plane;self.swing_height=swing_height;self.max_projection=.060
        self.joint_margin=.02;self.projection_iterations=14;self.projection_scan_intervals=16
        self.register_buffer('safe_lower',self.lower+self.joint_margin+1e-7)
        self.register_buffer('safe_upper',self.upper-self.joint_margin-1e-7)
        self.register_buffer('sensor_q',self.nominal.unsqueeze(0).expand(num_envs,-1).clone())
        self.register_buffer('sensor_up',torch.tensor([[0.,0.,1.]],**opts).expand(num_envs,-1).clone())

    @torch.jit.export
    def _original_reset(self, env_ids: Tensor) -> None:
        ids=env_ids.to(device=self.phase.device,dtype=torch.long)
        self.phase[ids]=0.;self.elapsed[ids]=0.;self.filters[ids]=0.;self.previous_clipped[ids]=0.

    @torch.jit.export
    def _sensor_observation(self, gyro:Tensor, gravity:Tensor, command:Tensor, q:Tensor, dq:Tensor, exposure_h:Tensor) -> Tensor:
        n=self.num_envs
        if gyro.shape != (n,3) or gravity.shape != (n,3) or command.shape != (n,3) or q.shape != (n,12) or dq.shape != (n,12) or exposure_h.shape != (n,12):raise ValueError('observation shape')
        if not bool(torch.isfinite(gyro).all() & torch.isfinite(gravity).all() & torch.isfinite(command).all() & torch.isfinite(q).all() & torch.isfinite(dq).all() & torch.isfinite(exposure_h).all()):raise ValueError('nonfinite observation')
        if bool((exposure_h<0).any() | (exposure_h>1).any()):raise ValueError('exposure outside [0,1]')
        gyro=gyro.to(self.phase);gravity=gravity.to(self.phase);command=command.to(self.phase);q=q.to(self.phase);dq=dq.to(self.phase);exposure_h=exposure_h.to(self.phase)
        obs=torch.cat([gyro*.25,gravity,command,q-self.nominal,dq*.05,self.previous_clipped,exposure_h,self.filters.reshape(n,9),torch.sin(2.*torch.pi*self.phase).unsqueeze(1),torch.cos(2.*torch.pi*self.phase).unsqueeze(1),torch.clamp(self.elapsed/self.bootstrap_s,max=1.).unsqueeze(1)],dim=1)
        return obs.to(self.output_anchor)

    def _step_impl(self,raw:Tensor,command:Tensor) -> Dict[str,Tensor]:
        n=self.num_envs
        if raw.shape != (n,12) or command.shape != (n,3):raise ValueError('controller shape')
        if not bool(torch.isfinite(raw).all() & torch.isfinite(command).all()):raise ValueError('nonfinite controller input')
        raw=raw.to(self.phase);cmd=command.to(self.phase)
        # Preserve the cardinal limits; additionally allow bounded pure yaw.
        # Float32 rounding tolerances match L09. Mixed raw commands are rejected;
        # independently decaying filters may legitimately contain an internal mix.
        translation=torch.linalg.vector_norm(cmd[:,:2],dim=1)>1e-9
        turning=torch.abs(cmd[:,2])>1e-9
        if bool((torch.abs(cmd[:,2])>.25+1e-8).any() | (cmd[:,0]>self.forward_command_limit+3e-8).any() | (cmd[:,0]<-.12-1e-8).any() | (torch.abs(cmd[:,1])>.12+1e-8).any() | ((torch.abs(cmd[:,0])>1e-9)&(torch.abs(cmd[:,1])>1e-9)).any() | (translation&turning).any()):raise ValueError('Outside registered L11 cardinal, pure-yaw or stop command domain')
        # Causal zero-order hold of the last IMU sample; observation never advances e.
        next_error=self.heading_error_rad+self.dt*(cmd[:,2]-self.sensor_yaw_rate)
        self.heading_error_rad.copy_(torch.atan2(torch.sin(next_error),torch.cos(next_error)))
        heading_correction=torch.where(translation&(~turning),torch.clamp(self.heading_gain*self.heading_error_rad,min=-self.heading_correction_limit,max=self.heading_correction_limit),torch.zeros_like(next_error))
        effective_yaw=cmd[:,2]+heading_correction
        active=(translation|turning).to(self.phase)
        desired=torch.cat([cmd[:,:2],active.unsqueeze(1)],dim=1)
        b=self.dt/self.filter_tau;a=0.8751733190429475  # exp(-.02/.15), checked against NumPy exact formula.
        delta=self.filters-desired.unsqueeze(1)
        f0=desired+a*delta[:,0]
        f1=desired+a*(delta[:,1]+b*delta[:,0])
        f2=desired+a*(delta[:,2]+b*delta[:,1]+.5*b*b*delta[:,0])
        self.filters.copy_(torch.stack([f0,f1,f2],dim=1))
        yaw_delta=self.yaw_filters-effective_yaw.unsqueeze(1)
        y0=effective_yaw+a*yaw_delta[:,0]
        y1=effective_yaw+a*(yaw_delta[:,1]+b*yaw_delta[:,0])
        y2=effective_yaw+a*(yaw_delta[:,2]+b*yaw_delta[:,1]+.5*b*b*yaw_delta[:,0])
        self.yaw_filters.copy_(torch.stack([y0,y1,y2],dim=1))
        self.phase.copy_(torch.remainder(self.phase+self.dt/self.period,1.))
        self.elapsed.add_(self.dt)
        progress=torch.clamp(self.elapsed/self.bootstrap_s,max=1.)
        boot=progress**3*(10.-15.*progress+6.*progress**2)
        u=torch.remainder(self.phase.unsqueeze(1)+self.offsets,1.)
        s=torch.clamp((u-self.duty)/(1.-self.duty),0.,1.)
        smooth=s**3*(10.-15.*s+6.*s**2)
        travel=self.period*(self.duty/2.-u+smooth)
        # Desired base twist evaluated at the stance-widened reference anchors.
        # During stance d(foot_in_body)/dt ~= -(v + omega cross anchor).
        # A zero filtered yaw gives the exact legacy cardinal arithmetic path.
        anchor_y=self.anchor[:,1].unsqueeze(0)+self.signs.unsqueeze(0)*self.stance_widen_m*boot.unsqueeze(1)
        yaw=self.yaw_filters[:,2].unsqueeze(1)
        vx=self.filters[:,2,0].unsqueeze(1)-yaw*anchor_y
        vy=self.filters[:,2,1].unsqueeze(1)+yaw*self.anchor[:,0].unsqueeze(0)
        foot_velocity=torch.stack([vx,vy],dim=2)
        xy=self.anchor[:,:2].unsqueeze(0)+boot[:,None,None]*travel.unsqueeze(2)*foot_velocity
        clearance=boot.unsqueeze(1)*self.filters[:,2,2].unsqueeze(1)*64.*.020*s**3*(1.-s)**3
        z=self.anchor[:,2].unsqueeze(0)+clearance
        feet=torch.cat([xy,z.unsqueeze(2)],dim=2)
        r=feet-self.origins.unsqueeze(0);x=r[:,:,0];y=r[:,:,1];zz=r[:,:,2]
        rad=y*y+zz*zz-.064**2;cosine=(x*x+rad-2.*.12**2)/(2.*.12**2)
        if bool((rad<=0).any() | (cosine < -1.).any() | (cosine > 1.).any()):raise RuntimeError('Reference IK failed; no hole filling or simulator state override')
        zs=-torch.sqrt(rad);calf=-torch.acos(cosine)
        hip=torch.remainder(torch.atan2(zz,y)-torch.atan2(zs,.064*self.signs)+torch.pi,2.*torch.pi)-torch.pi
        thigh=torch.remainder(torch.atan2(-x,-zs)-torch.atan2(.12*torch.sin(calf),.12+.12*torch.cos(calf))+torch.pi,2.*torch.pi)-torch.pi
        qref=torch.stack([hip,thigh,calf],dim=2).reshape(n,12)
        if bool((qref<self.lower-1e-12).any() | (qref>self.upper+1e-12).any()):raise RuntimeError('Reference outside physical q')
        clipped=self.residual_alpha*self.previous_clipped+(1.-self.residual_alpha)*torch.clamp(raw,-1.,1.);unclamped=qref+boot.unsqueeze(1)*self.scale*clipped
        target=torch.minimum(torch.maximum(unclamped,self.lower),self.upper)
        self.previous_clipped.copy_(clipped)
        return {'q_reference_rad':qref.to(self.output_anchor),'q_target_rad':target.to(self.output_anchor),'q_target_before_physical_clip_rad':unclamped.to(self.output_anchor),'physical_clip_delta_rad':(target-unclamped).to(self.output_anchor),'raw_action':raw.to(self.output_anchor),'clipped_action':clipped.to(self.output_anchor),'feet_relative_to_base_m':feet.to(self.output_anchor),'planned_center_clearance_m':clearance.to(self.output_anchor),'nominal_phase_stance':u<self.duty,'phase_after':self.phase.clone(),'filter_state_after':self.filters.clone(),'bootstrap_progress_after':progress,'bootstrap_blend_after':boot,'l10_heading_error_rad':self.heading_error_rad.clone(),'l10_measured_yaw_rate_rad_s':self.sensor_yaw_rate.clone(),'l10_raw_yaw_command_rad_s':cmd[:,2].clone(),'l10_heading_correction_rad_s':heading_correction,'l10_effective_yaw_command_rad_s':effective_yaw,'l10_filtered_yaw_rate_rad_s':self.yaw_filters[:,2].clone(),'l10_yaw_filter_state_after':self.yaw_filters.clone(),'l10_anchor_foot_velocity_m_s':foot_velocity}


    @torch.jit.export
    def reset(self,ids:Tensor)->None:
        self._original_reset(ids)
        self.yaw_filters[ids]=0.;self.heading_error_rad[ids]=0.;self.sensor_yaw_rate[ids]=0.
        self.sensor_q[ids]=self.nominal
        self.sensor_up[ids]=torch.tensor([0.,0.,1.],device=self.sensor_up.device,dtype=self.sensor_up.dtype)

    @torch.jit.export
    def observation(self,gyro:Tensor,gravity:Tensor,command:Tensor,q:Tensor,dq:Tensor,exposure_h:Tensor)->Tensor:
        obs=self._sensor_observation(gyro,gravity,command,q,dq,exposure_h)
        norm=torch.linalg.vector_norm(gravity,dim=1,keepdim=True)
        if bool((torch.abs(norm-1.)>.01).any()):raise ValueError('IMU gravity vector is not normalized')
        up=(-gravity/norm).to(self.sensor_up)
        denominator=up[:,1]**2+up[:,2]**2
        if bool((denominator<=1e-6).any()):raise ValueError('IMU heading rate singular near vertical pitch')
        rates=gyro.to(self.phase)
        yaw_rate=(up[:,1]*rates[:,1]+up[:,2]*rates[:,2])/denominator
        self.sensor_q.copy_(q.to(self.sensor_q));self.sensor_up.copy_(up)
        self.sensor_yaw_rate.copy_(yaw_rate)
        extra=torch.cat([self.yaw_filters,torch.sin(self.heading_error_rad).unsqueeze(1),torch.cos(self.heading_error_rad).unsqueeze(1)],dim=1)
        return torch.cat([obs,extra.to(self.output_anchor)],dim=1)

    @torch.jit.export
    def get_heading_error(self)->Tensor:
        return self.heading_error_rad.clone()

    def fk(self,q:Tensor)->Tensor:
        v=q.to(self.phase).reshape(self.num_envs,4,3);a=v[:,:,0];b=v[:,:,1];c=v[:,:,2]
        x=-.12*(torch.sin(b)+torch.sin(b+c));z=-.12*(torch.cos(b)+torch.cos(b+c))
        y=.064*self.signs.unsqueeze(0).expand(self.num_envs,-1)
        return torch.stack([x,torch.cos(a)*y-torch.sin(a)*z,torch.sin(a)*y+torch.cos(a)*z],dim=2)+self.origins.unsqueeze(0)

    def ik(self,feet:Tensor)->Tensor:
        r=feet-self.origins.unsqueeze(0);x=r[:,:,0];y=r[:,:,1];z=r[:,:,2]
        rad=y*y+z*z-.064**2;cosine=(x*x+rad-2*.12**2)/(2*.12**2)
        if bool((rad<=0).any()|(cosine < -1.).any()|(cosine>1.).any()):raise RuntimeError('L07 swing target outside IK domain')
        zs=-torch.sqrt(rad);calf=-torch.acos(cosine)
        hip=torch.remainder(torch.atan2(z,y)-torch.atan2(zs,.064*self.signs)+torch.pi,2*torch.pi)-torch.pi
        thigh=torch.remainder(torch.atan2(-x,-zs)-torch.atan2(.12*torch.sin(calf),.12+.12*torch.cos(calf))+torch.pi,2*torch.pi)-torch.pi
        return torch.stack([hip,thigh,calf],dim=2).reshape(self.num_envs,12)

    def feasible_ik(self,feet:Tensor):
        # Invalid analytic-domain lanes have finite placeholders only for masked
        # vector arithmetic. They can never be accepted as feasible targets.
        r=feet-self.origins.unsqueeze(0);x=r[:,:,0];y=r[:,:,1];z=r[:,:,2]
        rad=y*y+z*z-.064**2;cosine=(x*x+rad-2*.12**2)/(2*.12**2)
        domain=(rad>0.)&(cosine>=-1.)&(cosine<=1.)&torch.isfinite(feet).all(2)
        zs=-torch.sqrt(torch.clamp(rad,min=0.));calf=-torch.acos(torch.clamp(cosine,min=-1.,max=1.))
        hip=torch.remainder(torch.atan2(z,y)-torch.atan2(zs,.064*self.signs)+torch.pi,2*torch.pi)-torch.pi
        thigh=torch.remainder(torch.atan2(-x,-zs)-torch.atan2(.12*torch.sin(calf),.12+.12*torch.cos(calf))+torch.pi,2*torch.pi)-torch.pi
        q=torch.stack([hip,thigh,calf],dim=2)
        within=((q>=self.safe_lower.reshape(1,4,3)-2e-12)&(q<=self.safe_upper.reshape(1,4,3)+2e-12)).all(2)
        return q,domain&within,domain

    @torch.jit.export
    def step(self,raw:Tensor,command:Tensor)->Dict[str,Tensor]:
        d=self._step_impl(raw,command)
        bump=d['planned_center_clearance_m'].to(self.phase)/.020
        feet=d['feet_relative_to_base_m'].to(self.phase).clone()
        feet[:,:,2]+=bump*(self.swing_height-.020)
        feet[:,:,1]+=self.signs.unsqueeze(0)*self.stance_widen_m*d['bootstrap_blend_after'].to(self.phase).unsqueeze(1)
        ref=self.ik(feet)
        if bool(((ref<self.safe_lower)|(ref>self.safe_upper)).any()):
            raise RuntimeError('L13 reference itself violates registered 0.02 rad joint margins')
        residual=d['bootstrap_blend_after'].to(self.phase).unsqueeze(1)*self.scale*d['clipped_action'].to(self.phase)
        unbounded=ref+residual
        # One alpha for each complete leg, preserving the requested residual
        # direction. This is explicit contraction, not componentwise q clipping.
        safe_den=torch.where(torch.abs(residual)>1e-15,residual,torch.ones_like(residual))
        allowed=torch.where(residual>1e-15,(self.safe_upper-ref)/safe_den,
                    torch.where(residual< -1e-15,(self.safe_lower-ref)/safe_den,torch.ones_like(residual)))
        alpha=torch.clamp(allowed.reshape(self.num_envs,4,3).min(2).values,min=0.,max=1.)
        pre=ref+(residual.reshape(self.num_envs,4,3)*alpha.unsqueeze(2)).reshape(self.num_envs,12)
        start_feet=self.fk(pre)
        measured_feet=self.fk(self.sensor_q)
        plane=torch.min(torch.sum(measured_feet*self.sensor_up.unsqueeze(1),dim=2),dim=1).values
        height=torch.sum(start_feet*self.sensor_up.unsqueeze(1),dim=2)
        gap=torch.clamp(plane.unsqueeze(1)+self.swing_height-height,min=0.)
        requested=torch.zeros_like(bump);uncapped=torch.zeros_like(bump)
        if self.use_plane:
            uncapped=bump*gap
            requested=bump*torch.clamp(gap,max=self.max_projection)
        # File-only experiment: native fixed-four-leg scan/refine, preserving source postchecks below.
        native=torch.ops.sd_projection_fileonly_r1.project(pre.contiguous(),start_feet.contiguous(),self.sensor_up.contiguous(),requested.contiguous(),alpha.contiguous(),uncapped.contiguous())
        solved=native[0]
        endpoint_q=native[1]
        lo=native[2]
        first_invalid=native[3]
        hi=native[4]
        applied=native[5]
        endpoint_domain=native[6]
        endpoint_ok=native[7]
        active=~native[8]
        # This physical clip is an invariant backstop only. Any material change
        # is an error; all intentional modification is separately accounted.
        target=torch.minimum(torch.maximum(solved,self.lower),self.upper)
        if bool(((solved<self.lower+self.joint_margin-1e-10)|(solved>self.upper-self.joint_margin+1e-10)).any()):
            raise RuntimeError('L13 feasible target violated the registered joint margin')
        if bool(((target-solved).abs()>1e-12).any()):
            raise RuntimeError('L13 physical clip unexpectedly modified a feasible target')
        roundtrip=self.fk(solved)
        expected_feet=start_feet+applied.unsqueeze(2)*self.sensor_up.unsqueeze(1)
        if bool(((roundtrip-expected_feet).abs()>1e-9).any()):
            raise RuntimeError('L13 feasible projection FK/IK closure failed')
        # Reason bits: 1 residual joint margin, 2 requested IK domain,
        # 4 requested IK joint margin, 8 maximum projection distance.
        reasons=(alpha<1.-1e-12).to(torch.int64)+2*(~endpoint_domain).to(torch.int64)+4*((endpoint_domain&(~endpoint_ok))|(~active)).to(torch.int64)+8*(uncapped>requested+1e-12).to(torch.int64)
        d['q_reference_rad']=ref.to(self.output_anchor)
        d['q_target_rad']=target.to(self.output_anchor)
        d['q_target_before_physical_clip_rad']=solved.to(self.output_anchor)
        d['physical_clip_delta_rad']=(target-solved).to(self.output_anchor)
        d['feet_relative_to_base_m']=feet.to(self.output_anchor)
        d['planned_center_clearance_m']=(bump*self.swing_height).to(self.output_anchor)
        d['l07_projection_m']=applied.to(self.output_anchor)
        d['l07_preprojection_target_rad']=pre.to(self.output_anchor)
        d['l07_preprojection_clip_rad']=torch.zeros_like(pre).to(self.output_anchor)
        d['l07_plane_proxy_relative_m']=plane.to(self.output_anchor)
        d['l13_requested_residual_target_rad']=unbounded.to(self.output_anchor)
        d['l13_residual_scale_applied']=alpha.to(self.output_anchor)
        d['l13_residual_constraint_correction_rad']=(pre-unbounded).to(self.output_anchor)
        d['l13_projection_requested_m']=requested.to(self.output_anchor)
        d['l13_projection_request_before_cap_m']=uncapped.to(self.output_anchor)
        d['l13_projection_applied_m']=applied.to(self.output_anchor)
        d['l13_projection_unmet_m']=(uncapped-applied).to(self.output_anchor)
        d['l13_projection_fraction']=lo.to(self.output_anchor)
        d['l13_projection_first_invalid_fraction']=first_invalid.to(self.output_anchor)
        d['l13_projection_path_sample_gap_m']=(requested/float(self.projection_scan_intervals)).to(self.output_anchor)
        d['l13_constraint_reason_bits']=reasons
        d['l13_projection_path_rejected']=~active
        d['l13_projection_endpoint_domain_ok']=endpoint_domain
        d['l13_projection_endpoint_margin_ok']=endpoint_ok
        d['l13_target_foot_positions_m']=roundtrip.to(self.output_anchor)
        d['l13_target_joint_margin_rad']=torch.minimum(target-self.lower,self.upper-target).to(self.output_anchor)
        return d

    def _step_inputs(self,raw:Tensor,command:Tensor) -> Dict[str,Tensor]:
        n=self.num_envs
        if raw.shape != (n,12) or command.shape != (n,3):raise ValueError('controller shape')
        if not bool(torch.isfinite(raw).all() & torch.isfinite(command).all()):raise ValueError('nonfinite controller input')
        raw=raw.to(self.phase);cmd=command.to(self.phase)
        # Preserve the cardinal limits; additionally allow bounded pure yaw.
        # Float32 rounding tolerances match L09. Mixed raw commands are rejected;
        # independently decaying filters may legitimately contain an internal mix.
        translation=torch.linalg.vector_norm(cmd[:,:2],dim=1)>1e-9
        turning=torch.abs(cmd[:,2])>1e-9
        if bool((torch.abs(cmd[:,2])>.25+1e-8).any() | (cmd[:,0]>self.forward_command_limit+3e-8).any() | (cmd[:,0]<-.12-1e-8).any() | (torch.abs(cmd[:,1])>.12+1e-8).any() | ((torch.abs(cmd[:,0])>1e-9)&(torch.abs(cmd[:,1])>1e-9)).any() | (translation&turning).any()):raise ValueError('Outside registered L11 cardinal, pure-yaw or stop command domain')
        # Causal zero-order hold of the last IMU sample; observation never advances e.
        next_error=self.heading_error_rad+self.dt*(cmd[:,2]-self.sensor_yaw_rate)
        self.heading_error_rad.copy_(torch.atan2(torch.sin(next_error),torch.cos(next_error)))
        heading_correction=torch.where(translation&(~turning),torch.clamp(self.heading_gain*self.heading_error_rad,min=-self.heading_correction_limit,max=self.heading_correction_limit),torch.zeros_like(next_error))
        effective_yaw=cmd[:,2]+heading_correction
        active=(translation|turning).to(self.phase)
        desired=torch.cat([cmd[:,:2],active.unsqueeze(1)],dim=1)
        b=self.dt/self.filter_tau;a=0.8751733190429475  # exp(-.02/.15), checked against NumPy exact formula.
        delta=self.filters-desired.unsqueeze(1)
        f0=desired+a*delta[:,0]
        f1=desired+a*(delta[:,1]+b*delta[:,0])
        f2=desired+a*(delta[:,2]+b*delta[:,1]+.5*b*b*delta[:,0])
        self.filters.copy_(torch.stack([f0,f1,f2],dim=1))
        yaw_delta=self.yaw_filters-effective_yaw.unsqueeze(1)
        y0=effective_yaw+a*yaw_delta[:,0]
        y1=effective_yaw+a*(yaw_delta[:,1]+b*yaw_delta[:,0])
        y2=effective_yaw+a*(yaw_delta[:,2]+b*yaw_delta[:,1]+.5*b*b*yaw_delta[:,0])
        self.yaw_filters.copy_(torch.stack([y0,y1,y2],dim=1))
        self.phase.copy_(torch.remainder(self.phase+self.dt/self.period,1.))
        self.elapsed.add_(self.dt)
        progress=torch.clamp(self.elapsed/self.bootstrap_s,max=1.)
        boot=progress**3*(10.-15.*progress+6.*progress**2)
        u=torch.remainder(self.phase.unsqueeze(1)+self.offsets,1.)
        s=torch.clamp((u-self.duty)/(1.-self.duty),0.,1.)
        smooth=s**3*(10.-15.*s+6.*s**2)
        travel=self.period*(self.duty/2.-u+smooth)
        # Desired base twist evaluated at the stance-widened reference anchors.
        # During stance d(foot_in_body)/dt ~= -(v + omega cross anchor).
        # A zero filtered yaw gives the exact legacy cardinal arithmetic path.
        anchor_y=self.anchor[:,1].unsqueeze(0)+self.signs.unsqueeze(0)*self.stance_widen_m*boot.unsqueeze(1)
        yaw=self.yaw_filters[:,2].unsqueeze(1)
        vx=self.filters[:,2,0].unsqueeze(1)-yaw*anchor_y
        vy=self.filters[:,2,1].unsqueeze(1)+yaw*self.anchor[:,0].unsqueeze(0)
        foot_velocity=torch.stack([vx,vy],dim=2)
        xy=self.anchor[:,:2].unsqueeze(0)+boot[:,None,None]*travel.unsqueeze(2)*foot_velocity
        clearance=boot.unsqueeze(1)*self.filters[:,2,2].unsqueeze(1)*64.*.020*s**3*(1.-s)**3
        z=self.anchor[:,2].unsqueeze(0)+clearance
        feet=torch.cat([xy,z.unsqueeze(2)],dim=2)
        r=feet-self.origins.unsqueeze(0);x=r[:,:,0];y=r[:,:,1];zz=r[:,:,2]
        rad=y*y+zz*zz-.064**2;cosine=(x*x+rad-2.*.12**2)/(2.*.12**2)
        if bool((rad<=0).any() | (cosine < -1.).any() | (cosine > 1.).any()):raise RuntimeError('Reference IK failed; no hole filling or simulator state override')
        zs=-torch.sqrt(rad);calf=-torch.acos(cosine)
        hip=torch.remainder(torch.atan2(zz,y)-torch.atan2(zs,.064*self.signs)+torch.pi,2.*torch.pi)-torch.pi
        thigh=torch.remainder(torch.atan2(-x,-zs)-torch.atan2(.12*torch.sin(calf),.12+.12*torch.cos(calf))+torch.pi,2.*torch.pi)-torch.pi
        qref=torch.stack([hip,thigh,calf],dim=2).reshape(n,12)
        if bool((qref<self.lower-1e-12).any() | (qref>self.upper+1e-12).any()):raise RuntimeError('Reference outside physical q')
        clipped=self.residual_alpha*self.previous_clipped+(1.-self.residual_alpha)*torch.clamp(raw,-1.,1.);unclamped=qref+boot.unsqueeze(1)*self.scale*clipped
        target=torch.minimum(torch.maximum(unclamped,self.lower),self.upper)
        self.previous_clipped.copy_(clipped)
        return {'clipped_action':clipped.to(self.output_anchor),'feet_relative_to_base_m':feet.to(self.output_anchor),'planned_center_clearance_m':clearance.to(self.output_anchor),'bootstrap_blend_after':boot}

    @torch.jit.export
    def step_target(self,raw:Tensor,command:Tensor)->Tensor:
        d=self._step_inputs(raw,command)
        bump=d['planned_center_clearance_m'].to(self.phase)/.020
        feet=d['feet_relative_to_base_m'].to(self.phase).clone()
        feet[:,:,2]+=bump*(self.swing_height-.020)
        feet[:,:,1]+=self.signs.unsqueeze(0)*self.stance_widen_m*d['bootstrap_blend_after'].to(self.phase).unsqueeze(1)
        ref=self.ik(feet)
        if bool(((ref<self.safe_lower)|(ref>self.safe_upper)).any()):
            raise RuntimeError('L13 reference itself violates registered 0.02 rad joint margins')
        residual=d['bootstrap_blend_after'].to(self.phase).unsqueeze(1)*self.scale*d['clipped_action'].to(self.phase)
        unbounded=ref+residual
        # One alpha for each complete leg, preserving the requested residual
        # direction. This is explicit contraction, not componentwise q clipping.
        safe_den=torch.where(torch.abs(residual)>1e-15,residual,torch.ones_like(residual))
        allowed=torch.where(residual>1e-15,(self.safe_upper-ref)/safe_den,
                    torch.where(residual< -1e-15,(self.safe_lower-ref)/safe_den,torch.ones_like(residual)))
        alpha=torch.clamp(allowed.reshape(self.num_envs,4,3).min(2).values,min=0.,max=1.)
        pre=ref+(residual.reshape(self.num_envs,4,3)*alpha.unsqueeze(2)).reshape(self.num_envs,12)
        start_feet=self.fk(pre)
        measured_feet=self.fk(self.sensor_q)
        plane=torch.min(torch.sum(measured_feet*self.sensor_up.unsqueeze(1),dim=2),dim=1).values
        height=torch.sum(start_feet*self.sensor_up.unsqueeze(1),dim=2)
        gap=torch.clamp(plane.unsqueeze(1)+self.swing_height-height,min=0.)
        requested=torch.zeros_like(bump);uncapped=torch.zeros_like(bump)
        if self.use_plane:
            uncapped=bump*gap
            requested=bump*torch.clamp(gap,max=self.max_projection)
        # File-only experiment: native fixed-four-leg scan/refine, preserving source postchecks below.
        native=torch.ops.sd_projection_fileonly_r1.project(pre.contiguous(),start_feet.contiguous(),self.sensor_up.contiguous(),requested.contiguous(),alpha.contiguous(),uncapped.contiguous())
        solved=native[0]
        endpoint_q=native[1]
        lo=native[2]
        first_invalid=native[3]
        hi=native[4]
        applied=native[5]
        endpoint_domain=native[6]
        endpoint_ok=native[7]
        active=~native[8]
        # This physical clip is an invariant backstop only. Any material change
        # is an error; all intentional modification is separately accounted.
        target=torch.minimum(torch.maximum(solved,self.lower),self.upper)
        if bool(((solved<self.lower+self.joint_margin-1e-10)|(solved>self.upper-self.joint_margin+1e-10)).any()):
            raise RuntimeError('L13 feasible target violated the registered joint margin')
        if bool(((target-solved).abs()>1e-12).any()):
            raise RuntimeError('L13 physical clip unexpectedly modified a feasible target')
        roundtrip=self.fk(solved)
        expected_feet=start_feet+applied.unsqueeze(2)*self.sensor_up.unsqueeze(1)
        if bool(((roundtrip-expected_feet).abs()>1e-9).any()):
            raise RuntimeError('L13 feasible projection FK/IK closure failed')
        return target.to(self.output_anchor)

    def forward(self,raw:Tensor,command:Tensor)->Dict[str,Tensor]:
        return self.step(raw,command)
