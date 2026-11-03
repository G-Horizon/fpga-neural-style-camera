from pathlib import Path
import os,subprocess,json,hashlib,sys
ESP=Path(__file__).resolve().parents[1];OUT=Path(os.environ.get('STYLE_TEST_RESULTS',str(ESP/'tests/results')));OUT.mkdir(exist_ok=True)
cases={
 'uart_wire':['a_uart_wire.c','@test_a_uart_wire.c'],
 'uart_adapter':['a_uart_wire.c','a_uart.c','@test_a_uart_adapter.c'],
 'frame_pool':['frame_pool.c','@test_frame_pool.c'],
 'stream_session':['frame_pool.c','@test_stream_session.c'],
 'remote_control':['remote_control.c','@test_remote_control.c'],
 'remote_status':['remote_control.c','remote_json.c','@test_remote_status.c'],
 'spi_v2':['a_uart_wire.c','wire_protocol.c','fpga_transport.c','@test_spi_v2.c'],
 'stream_fixture':['wire_protocol.c','@make_stream_fixture.c']}
results=[]
def run(cmd,label):
 p=subprocess.run(list(map(str,cmd)),cwd=ESP,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace');(OUT/(label+'.log')).write_text(p.stdout,encoding='utf-8');assert p.returncode==0,p.stdout;return p.stdout
for name,files in cases.items():
 paths=[ESP/('tests/'+x[1:]if x.startswith('@')else'main/'+x)for x in files];exe=OUT/(name+'.exe')
 run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-I'+str(ESP/'tests/stubs'),'-I'+str(ESP/'main'),*paths,'-o',exe],name+'_compile')
 text=run([exe,*([OUT]if name in ['stream_fixture','stream_session']else[])],name+'_run');assert'PASS:'in text;print(text.strip(),flush=True)
 results.append(dict(case=name,result='PASS',log_sha256=hashlib.sha256((OUT/(name+'_run.log')).read_bytes()).hexdigest(),sources=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())for p in paths]))
text=run(['node',ESP/'tests/test_mobile.mjs'],'mobile');assert'PASS:'in text;print(text.strip())
doc=dict(result='PASS',cases=results,mobile='MOBILE_RESULTS.json',scope='ProductionCwire/adapter/pool/commandcore+fakephysicalAPI/AXIpeersandCtoJSwire. NotphysicalSPI/UART/I2S/WiFi/FPGAorvoiceaccuracy.',bindings=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())for folder in ['main','tests/stubs']for p in (ESP/folder).rglob('*')if p.is_file()])
(OUT/'HOST_FINAL.json').write_text(json.dumps(doc,indent=2)+'\n');print('PASS: stable production source host gates bound')
