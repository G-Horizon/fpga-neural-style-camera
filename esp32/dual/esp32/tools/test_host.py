from pathlib import Path
import os,subprocess,json,hashlib
ESP=Path(__file__).resolve().parents[1];OUT=Path(os.environ.get('STYLE_TEST_RESULTS',str(ESP/'tests/results')));OUT.mkdir(exist_ok=True)
exe=OUT/'test_a_uart_wire.exe';files=[ESP/'main/a_uart_wire.c',ESP/'tests/test_a_uart_wire.c']
cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-I'+str(ESP/'main'),*map(str,files),'-o',str(exe)]
p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True);(OUT/'uart_compile.log').write_text(p.stdout);assert p.returncode==0,p.stdout
p=subprocess.run([str(exe)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True);(OUT/'uart_test.log').write_text(p.stdout);assert p.returncode==0 and'PASS:'in p.stdout,p.stdout
(OUT/'UART_RESULTS.json').write_text(json.dumps(dict(result='PASS',scope='Pure production UART wire/parser and host pending/session state,notphysicalUARTorFPGAserver',sources=[dict(path=str(f),sha256=hashlib.sha256(f.read_bytes()).hexdigest())for f in files]),indent=2)+'\n');print(p.stdout)
