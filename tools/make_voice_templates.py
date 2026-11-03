"""Build a replacement template header from four separate mono PCM16 16kHz WAVs."""
from pathlib import Path
import argparse
from voice_features import read_pcm,extract,segments,normalize_time,WORDS
def main():
 p=argparse.ArgumentParser(description=__doc__)
 for word in WORDS:p.add_argument('--'+word,type=Path,required=True)
 p.add_argument('--output',type=Path,required=True)
 a=p.parse_args();templates=[];lengths=[]
 for word in WORDS:
  features,amp=extract(read_pcm(getattr(a,word)));spans=segments(amp)
  if len(spans)!=1:p.error(word+': expected exactly one voiced phrase')
  start,end=spans[0];length=end-start
  if not 10<=length<=64:p.error(word+': voiced duration must be 10..64 feature frames')
  lengths.append(length);templates.append(normalize_time(features[start:end]).tolist())
 lines=['#ifndef STYLE_VOICE_TEMPLATES_H','#define STYLE_VOICE_TEMPLATES_H',
  '/* Locally generated replacement; enable only after fresh positive/negative tests. */',
  'static const uint8_t training_lengths[4] = {'+','.join(map(str,lengths))+'};',
  'static const uint8_t templates[4][16][8] = {']
 for matrix in templates:lines.append('  {'+','.join('{'+','.join(map(str,row))+'}'for row in matrix)+'},')
 lines+=['};','#endif',''];a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text('\n'.join(lines),encoding='utf-8')
 print('Generated replacement header. Replace the original arrays with an include, then validate with fresh utterances and negatives.')
if __name__=='__main__':main()
