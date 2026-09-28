#!/usr/bin/env python3
"""Synthesize a saved person report in Mandarin and optionally play it."""
import argparse
import ctypes
import ctypes.util
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import wave


def synthesize(text, output):
    library = ctypes.util.find_library('espeak-ng')
    if not library:
        raise RuntimeError('libespeak-ng is required for Mandarin speech')
    lib = ctypes.CDLL(library)
    callback_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_short),
                                    ctypes.c_int, ctypes.c_void_p)
    chunks = []

    @callback_type
    def callback(samples, count, _events):
        if samples and count > 0:
            chunks.append(ctypes.string_at(samples, count*2))
        return 0

    lib.espeak_Initialize.argtypes = [ctypes.c_int, ctypes.c_int,
                                      ctypes.c_char_p, ctypes.c_int]
    lib.espeak_SetSynthCallback.argtypes = [callback_type]
    lib.espeak_SetVoiceByName.argtypes = [ctypes.c_char_p]
    lib.espeak_Synth.argtypes = [ctypes.c_void_p, ctypes.c_size_t,
        ctypes.c_uint, ctypes.c_int, ctypes.c_uint, ctypes.c_uint,
        ctypes.POINTER(ctypes.c_uint), ctypes.c_void_p]
    rate = lib.espeak_Initialize(2, 0, None, 0x8000)
    if rate <= 0:
        raise RuntimeError('eSpeak initialization failed')
    try:
        lib.espeak_SetSynthCallback(callback)
        if lib.espeak_SetVoiceByName(b'cmn') != 0:
            raise RuntimeError('Mandarin voice cmn is missing')
        lib.espeak_SetParameter(1, 155, 0)
        encoded = text.encode('utf-8') + b'\0'
        buffer = ctypes.create_string_buffer(encoded)
        if lib.espeak_Synth(buffer, len(encoded), 0, 1, 0, 0x1001,
                            None, None) != 0:
            raise RuntimeError('Mandarin synthesis failed')
        lib.espeak_Synchronize()
    finally:
        lib.espeak_Terminate()
    pcm = b''.join(chunks)
    if not pcm or not any(pcm):
        raise RuntimeError('speech engine returned empty/silent PCM')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(output), 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(rate)
        wav.writeframes(pcm)
    return len(pcm)/2.0/rate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--play', action='store_true')
    args = parser.parse_args()
    path = Path(args.report)
    report = json.loads(path.read_text(encoding='utf-8'))
    status = {'report_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'text': report['speech_text'], 'synthesis_pass': False,
              'playback_requested': args.play, 'playback_pass': False}
    try:
        status['duration_s'] = synthesize(report['speech_text'], args.output)
        status['synthesis_pass'] = True
        if args.play:
            player = shutil.which('pw-play') or shutil.which('aplay')
            if not player:
                raise RuntimeError('pw-play/aplay is required for speech playback')
            process = subprocess.run([player, args.output], timeout=90, check=False)
            status['playback_pass'] = process.returncode == 0
            if not status['playback_pass']:
                raise RuntimeError('audio playback returned %d' % process.returncode)
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        status['error'] = str(error)
    Path(args.output).with_suffix('.status.json').write_text(
        json.dumps(status, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(status, ensure_ascii=False))
    return 0 if status['synthesis_pass'] and (not args.play or status['playback_pass']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
