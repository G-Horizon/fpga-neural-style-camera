"""Invoke ESP-IDF through an already exported SDK environment; no local paths."""
from pathlib import Path
import argparse,os,subprocess,sys
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--idf-path',default=os.environ.get('IDF_PATH'))
 p.add_argument('--project',type=Path,required=True)
 a,extra=p.parse_known_args()
 if not a.idf_path:p.error('Activate the ESP-IDF export environment or supply --idf-path')
 target=Path(a.idf_path)/'tools/idf.py'
 if not target.is_file():p.error('ESP-IDF tools/idf.py was not found')
 extra=extra[1:]if extra[:1]==['--']else extra
 if not extra:extra=['build']
 raise SystemExit(subprocess.call([sys.executable,str(target),*extra],cwd=a.project.resolve()))
if __name__=='__main__':main()
