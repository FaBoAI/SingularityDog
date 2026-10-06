"""Pinned saved-output CPU comparison only; default PLAN does not import Torch."""
import argparse, datetime, json, math, statistics, struct, time
from pathlib import Path
from . import conversion as c

FIELDS=('q_target_rad_diagnostic_only','actor_residual12','observation74')
WIDTHS=(12,12,74)
def pairs(rows):
    out={}
    for key,value in rows:c.require(key not in out,'Duplicate JSON key');out[key]=value
    return out
def parse(raw):return json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
def bits(rows):return tuple(tuple(struct.pack('<d',v)for v in row)for row in rows)
def distribution(values):
    c.require(type(values)is list and values and all(type(v)is int and v>0 for v in values),'Positive exact raw times required')
    values=sorted(values);count=len(values)
    return {'count':count,'median_us':statistics.median(values)/1000,'p95_us':values[math.ceil(.95*count)-1]/1000,'p99_us':values[math.ceil(.99*count)-1]/1000,'max_us':values[-1]/1000}
def saved_rows(args):
    report_raw=c.read(args.report,args.report_sha256);record_raw=c.read(args.records,args.records_sha256);audit_raw=c.read(args.raw_audit,args.raw_audit_sha256)
    report,records,audit=map(parse,(report_raw,record_raw,audit_raw))
    c.require(report['status']=='COMPLETE_DIAGNOSTIC' and type(report['cycles_completed'])is int and report['cycles_completed']==501 and report['errors']==[] and report['motor_enable_sent']is False and report['learned_targets_sent']is False,'Complete disabled 501 source required')
    c.require(audit['status']=='PASS_RAW_EVIDENCE' and type(audit['cycles_audited'])is int and audit['cycles_audited']==501 and all(pin in audit['input_file_sha256'].values()for pin in (args.report_sha256,args.records_sha256)),'Report/records independent raw audit binding required')
    c.require(type(records)is list and len(records)==501,'Exact saved 501 required');frames=[]
    for index,row in enumerate(records):
        c.require(type(row)is dict and type(row.get('cycle'))is int and row['cycle']==index+1,'Saved order must be 1..501')
        observed=row.get('observed');c.require(type(observed)is dict and observed.get('status')=='TICK_OBSERVED_NO_OUTPUT' and observed.get('output_allowed')is False,'Saved no-output observation required')
        values=[]
        for field,width in zip(FIELDS,WIDTHS):
            value=observed.get(field)
            c.require(type(value)is list and len(value)==width and all(type(v)is float and math.isfinite(v)for v in value),'Exact finite saved float rows required')
            c.require(all(struct.pack('<d',v)==struct.pack('<d',struct.unpack('<f',struct.pack('<f',v))[0])for v in value),'Saved values must be exact widened float32')
            values.append(value)
        frames.append(tuple(values))
    return frames
def compare(torch,reference,native,frames):
    tensors=[tuple(torch.tensor([row],dtype=torch.float32)for row in values)for values in frames]
    aggregate={name:{'raw_wall_ns':[],'raw_thread_cpu_ns':[]}for name in ('reference_python','native_row')};blocks=[]
    with torch.inference_mode():
        for order in (False,True,True,False):
            rows={name:{'raw_wall_ns':[],'raw_thread_cpu_ns':[]}for name in aggregate};orders=[]
            for inputs in tensors[:10]:
                for rowfn in (reference.row,native.row):reference.outputs(inputs[0],lambda:inputs[1],lambda:inputs[2],rowfn)
            for index,(inputs,golden)in enumerate(zip(tensors,frames)):
                before=[value.view(torch.int32).clone()for value in inputs]
                names=('native_row','reference_python')if (bool(index%2)!=order)else('reference_python','native_row');orders.append(list(names))
                actor_getter=lambda:inputs[1];observation_getter=lambda:inputs[2]
                for name in names:
                    rowfn=reference.row if name=='reference_python'else native.row
                    wb,cb=time.perf_counter_ns(),time.thread_time_ns()
                    result=reference.outputs(inputs[0],actor_getter,observation_getter,rowfn)
                    ce,we=time.thread_time_ns(),time.perf_counter_ns()
                    rows[name]['raw_wall_ns'].append(we-wb);rows[name]['raw_thread_cpu_ns'].append(ce-cb)
                    c.require(bits(result)==bits(golden),'Saved output float64 bits differ')
                    c.require(all(torch.equal(prior,value.view(torch.int32))for prior,value in zip(before,inputs)),'Tensor input bits changed')
            for name,row in rows.items():
                for key in ('raw_wall_ns','raw_thread_cpu_ns'):aggregate[name][key]+=row[key]
                row['wall']=distribution(row['raw_wall_ns']);row['thread_cpu']=distribution(row['raw_thread_cpu_ns'])
            blocks.append({'block':len(blocks)+1,'candidate_first_index_zero':order,'raw_call_orders':orders,'timing':rows})
    for row in aggregate.values():row['wall']=distribution(row['raw_wall_ns']);row['thread_cpu']=distribution(row['raw_thread_cpu_ns'])
    return {'blocks':blocks,'aggregate':aggregate,'saved_values_per_call':98,'checks_outside_timing':True,'source_order':['AB','BA','BA','AB'],'full_model_or_live_loop_measured':False}
def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('report','records','raw-audit'):parser.add_argument('--'+name,required=True);parser.add_argument('--'+name+'-sha256',required=True)
    parser.add_argument('--observer',default=str(c.OBSERVER));parser.add_argument('--shadow',default=str(c.SHADOW));parser.add_argument('--build-helper',default=str(c.BUILD_HELPER));parser.add_argument('--output',required=True);parser.add_argument('--execute-file-only',action='store_true');args=parser.parse_args(argv)
    output=c.fresh(args.output);reference=c.Reference(args.observer,args.shadow);frames=saved_rows(args)
    module_raw=c.read(Path(__file__).absolute());conversion_raw=c.read(c.HERE/'conversion.py');cpp=c.read(c.HERE/'rows.cpp');c.read(args.build_helper,c.BUILD_HELPER_SHA)
    result={'schema':'private.saved-output-row-conversion-profile.v1','status':'FILE_ONLY_PLAN','generated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'report_sha256':args.report_sha256,'records_sha256':args.records_sha256,'raw_audit_sha256':args.raw_audit_sha256,'source_sha256':{'saved_profile.py':c.sha(module_raw),'conversion.py':c.sha(conversion_raw),'rows.cpp':c.sha(cpp),'reference_observer':c.OBSERVER_SHA,'reference_shadow':c.SHADOW_SHA,'build_helper':c.BUILD_HELPER_SHA},'saved_frames':501,'values_per_consume':98,'model_loaded':False,'native_library_loaded':False,'hardware_opened':False,'output_allowed':False,'runtime_selection_added':False,'active_controller_qualification':False,'whole_20ms_loop_gain_proven':False}
    if args.execute_file_only:
        import torch
        torch.set_num_threads(1);torch.set_num_interop_threads(1)
        record=c.build(torch,output.with_name(output.stem+'-rows.so'),args.build_helper);native=c.NativeRows(torch,record['library_path'],record['library_sha256'],record,reference)
        result['paired_conversion_timing']=compare(torch,reference,native,frames);native.verify();result.update(status='PASS_FILE_ONLY_SAVED_OUTPUT_ROW_COMPARE',native_library_loaded=True,build=record,saved_all_float64_bits_exact=True,input_float32_bits_preserved=True,native_calls=native.native_calls,fallback_calls=native.fallback_calls,environment={'torch_version':torch.__version__})
    reference.verify();c.read(args.build_helper,c.BUILD_HELPER_SHA)
    for name,pin in ((args.report,args.report_sha256),(args.records,args.records_sha256),(args.raw_audit,args.raw_audit_sha256)):c.read(name,pin)
    c.require(c.read(Path(__file__).absolute())==module_raw and c.read(c.HERE/'conversion.py')==conversion_raw and c.read(c.HERE/'rows.cpp')==cpp,'Experiment source changed')
    with output.open('x')as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({'status':result['status'],'output_allowed':False,'output':str(output)}));return result
if __name__=='__main__':main()
