"""Local, integer-exact personal keyword prototype; no ASR or remote upload.

Eight Goertzel bins are a small speaker-template prototype, not full ASR.
This module contains reusable extraction/classification algorithms.
"""
from pathlib import Path
import argparse, hashlib, json, math, wave
import numpy as np

N = 256
FREQS = (375, 625, 1000, 1500, 2250, 3250, 4500, 6000)
COEFS = tuple(round(2 * math.cos(2 * math.pi * f / 16000) * 16384) for f in FREQS)
WORDS = ('vangogh', 'ukiyoe', 'inkwash', 'next')
VAD_THRESHOLD = 80
HANG = 6
MAX_SCORE = 650
MIN_MARGIN = 40

def read_pcm(path):
    with wave.open(str(path)) as w:
        if (w.getnchannels(), w.getsampwidth(), w.getframerate()) != (1, 2, 16000):
            raise ValueError('Expected 16 kHz signed 16 bit mono WAV')
        return np.frombuffer(w.readframes(w.getnframes()), dtype='<i2').astype(np.int32)

def frame_feature(samples):
    """Same rounding, shared-multiplier staging and saturation as feature RTL."""
    logs = []
    scaled = samples >> 6
    for c in COEFS:
        s1 = s2 = 0
        for value in scaled:
            s = int(value) + ((c * s1) >> 14) - s2
            s = min(16383, max(-16383, s))
            s2, s1 = s1, s
        cross = (s1 * s2) >> 14
        power = max(0, s1 * s1 + s2 * s2 - c * cross)
        logs.append(power.bit_length())
    maximum = max(logs)
    return [maximum - v for v in logs]

def extract(pcm):
    pcm = pcm[:len(pcm)//N*N].reshape(-1, N)
    features = np.asarray([frame_feature(f) for f in pcm], dtype=np.uint8)
    amp = np.abs(pcm.astype(np.int64)).sum(axis=1) >> 8
    return features, amp

def segments(amp):
    result, start, last, quiet = [], None, None, 0
    for i, v in enumerate(amp > VAD_THRESHOLD):
        if v:
            if start is None: start = i
            last, quiet = i, 0
        elif start is not None:
            quiet += 1
            if quiet == HANG:
                result.append((start, last+1)); start = None; quiet = 0
    if start is not None: result.append((start, last+1))
    return result

def normalize_time(f):
    # Nearest frame at sixteen equal temporal-bin centers; only shift/add in RTL.
    return np.asarray([f[((2*t+1)*len(f)) >> 5] for t in range(16)], dtype=np.int32)

def classify(f, templates, lengths):
    if len(f) < 10 or len(f) > 64:
        return {'accepted': False, 'reason': 'duration', 'frames': len(f)}
    n = normalize_time(f)
    scores = [int(np.abs(n - t).sum()) if max(10, l//2) <= len(f) <= min(64, l*2) else 65535
              for t,l in zip(templates,lengths)]
    ranking = sorted(range(4),key=lambda i:scores[i])
    best, second = ranking[:2]
    accepted = scores[best] <= MAX_SCORE and scores[second]-scores[best] >= MIN_MARGIN
    return {'accepted': accepted, 'class': best, 'word': WORDS[best],
            'score': scores[best], 'second_score': scores[second], 'scores': scores,
            'frames': len(f), 'reason': 'match' if accepted else 'score_or_margin'}

