import builtins,json,tempfile,types,unittest
from pathlib import Path
from unittest import mock
from . import saved_profile as s

class SavedProfileTests(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name).resolve()
    def tearDown(self):self.temp.cleanup()
    def fixture(self,change=None):
        report={'status':'COMPLETE_DIAGNOSTIC','cycles_completed':501,'errors':[],'motor_enable_sent':False,'learned_targets_sent':False}
        rows=[{'cycle':i+1,'observed':{'status':'TICK_OBSERVED_NO_OUTPUT','output_allowed':False,'q_target_rad_diagnostic_only':[0.,0.,-1.]*4,'actor_residual12':[-0.]*12,'observation74':[0.]*74}}for i in range(501)]
        if change:change(report,rows)
        args={}
        for name,data in (('report',report),('records',rows)):
            p=self.root/(name+'.json');p.write_text(json.dumps(data));args[name]=str(p);args[name+'_sha256']=s.c.sha(p.read_bytes())
        audit={'status':'PASS_RAW_EVIDENCE','cycles_audited':501,'input_file_sha256':{name:args[name+'_sha256']for name in ('report','records')}}
        p=self.root/'audit.json';p.write_text(json.dumps(audit));args['raw_audit']=str(p);args['raw_audit_sha256']=s.c.sha(p.read_bytes());return types.SimpleNamespace(**args)
    def argv(self,args):return [item for name in ('report','records','raw-audit')for item in ('--'+name,getattr(args,name.replace('-','_')),'--'+name+'-sha256',getattr(args,name.replace('-','_')+'_sha256'))]+['--output',str(self.root/'plan.json')]

    def test_plan_imports_no_torch_and_opens_no_library(self):
        args=self.fixture();original=builtins.__import__
        def blocked(name,*a,**k):
            if name=='torch' or name.startswith('torch.'):raise AssertionError('Torch import')
            return original(name,*a,**k)
        with mock.patch('builtins.__import__',side_effect=blocked),mock.patch.object(s.c,'build',side_effect=AssertionError('compile')),mock.patch.object(s.c,'NativeRows',side_effect=AssertionError('native load')):
            result=s.main(self.argv(args))
        self.assertEqual(result['status'],'FILE_ONLY_PLAN');self.assertFalse(result['model_loaded']);self.assertFalse(result['native_library_loaded']);self.assertEqual(result['values_per_consume'],98)

    def test_saved_order_count_and_typed_complete_count(self):
        for change in (lambda r,x:x.pop(),lambda r,x:x[0].update(cycle=True),lambda r,x:x[1].update(cycle=1),lambda r,x:r.update(cycles_completed=501.0)):
            args=self.fixture(change)
            with self.assertRaises(ValueError):s.saved_rows(args)

    def test_saved_bool_nonfinite_and_non_float32_values_reject(self):
        for value in (True,float('nan'),float('inf'),.1):
            args=self.fixture(lambda report,rows:rows[0]['observed']['actor_residual12'].__setitem__(0,value))
            with self.assertRaises(ValueError):s.saved_rows(args)

    def test_audit_mismatch_and_failed_source_never_become_plan(self):
        args=self.fixture(lambda r,x:r.update(status='ABORTED'))
        with self.assertRaises(ValueError):s.main(self.argv(args))
        self.assertFalse((self.root/'plan.json').exists())
        args=self.fixture();p=Path(args.raw_audit);audit=json.loads(p.read_bytes());audit['input_file_sha256']['records']='a'*64;p.write_text(json.dumps(audit));args.raw_audit_sha256=s.c.sha(p.read_bytes())
        with self.assertRaises(ValueError):s.saved_rows(args)

    def test_json_duplicate_and_sha_mismatch_reject(self):
        with self.assertRaisesRegex(ValueError,'Duplicate'):s.parse('{"a":1,"a":2}')
        args=self.fixture();args.records_sha256='a'*64
        with self.assertRaises(ValueError):s.saved_rows(args)

    def test_source_signedzero_bits_and_exact_widths(self):
        args=self.fixture();frames=s.saved_rows(args);self.assertEqual(len(frames),501);self.assertEqual([len(v)for v in frames[0]],[12,12,74])
        import struct
        self.assertEqual(s.bits(frames[0])[1][0],struct.pack('<d',-0.))
        args=self.fixture(lambda r,x:x[0]['observed']['observation74'].pop())
        with self.assertRaises(ValueError):s.saved_rows(args)

    def test_existing_symlink_and_git_output_reject(self):
        out=self.root/'out.json';out.write_text('keep')
        with self.assertRaises(ValueError):s.c.fresh(out)
        out.unlink();out.symlink_to(self.root/'new')
        with self.assertRaises(ValueError):s.c.fresh(out)
        out.unlink();(self.root/'.git').mkdir()
        with self.assertRaises(ValueError):s.c.fresh(out)

    def test_cli_abbreviation_and_execute_command_injection_reject(self):
        with self.assertRaises(SystemExit):s.main(['--rep','x'])
        args=self.fixture()
        with self.assertRaises(SystemExit):s.main(self.argv(args)+['--command','anything'])

if __name__=='__main__':unittest.main()
