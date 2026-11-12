#!/usr/bin/env python3
"""Turn the frames saved by scripts/record_media.py into the README visuals.

Writes to OUT (default docs/media/):
  demo.gif      chase camera over a window of the mission, with a small HUD (sim time, state, apples)
  overview.jpg  whole orchard with the robot at work
  closeup.jpg   robot next to apples while picking
  depot.jpg     robot unloading at the depot
Frames are chosen from the mission state logged in events.jsonl; override with --*-t (sim seconds).

Usage: scripts/make_media.py [--raw artifacts/media_raw] [--out docs/media]
                             [--gif-start 0] [--gif-end 240] [--gif-step 2.0] [--fps 10]
"""
import argparse
import bisect
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def load_frames(raw, cam):
    d = os.path.join(raw, cam)
    return sorted((int(f[:-4]) / 1000.0, os.path.join(d, f)) for f in os.listdir(d) if f.endswith('.jpg'))


def load_stats(raw):
    stats = []
    with open(os.path.join(raw, 'events.jsonl'), encoding='utf-8') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] == 'stats':
                stats.append(e)
    return stats


def stats_at(stats, t):
    times = [s['t'] for s in stats]
    i = bisect.bisect_right(times, t) - 1
    return stats[i] if i >= 0 else {}


def frame_at(frames, t):
    return min(frames, key=lambda f: abs(f[0] - t))


def first_t(stats, state, after=0.0, last=False):
    hits = [s['t'] for s in stats if s.get('state') == state and s['t'] >= after]
    if not hits:
        return None
    return hits[-1] if last else hits[0]


def font(size):
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def hud(img, text, size):
    """Dark translucent band at the top with white text."""
    img = img.convert('RGBA')
    band = Image.new('RGBA', img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(band)
    h = int(size * 1.7)
    draw.rectangle((0, 0, img.width, h), fill=(0, 0, 0, 140))
    draw.text((int(size * 0.6), int(size * 0.3)), text, font=font(size), fill=(255, 255, 255, 255))
    return Image.alpha_composite(img, band).convert('RGB')


def make_gif(frames, stats, out, start, end, step, fps, width):
    picked, t, sel = [], start, []
    while t <= end:
        sel.append(frame_at(frames, t))
        t += step
    speed = step * fps
    for ft, path in sel:
        s = stats_at(stats, ft)
        img = Image.open(path)
        img = img.resize((width, round(img.height * width / img.width)), Image.LANCZOS)
        text = (f"t = {ft:4.0f} s   {s.get('state', ''):<9}  picked {s.get('picked', 0):>2}   "
                f"deposited {s.get('unloaded', 0):>2}   ({speed:.0f}x)")
        picked.append(hud(img, text, max(11, width // 48)))
    # One palette for all frames, from a mosaic of frames plus apple reds (rare pixels the
    # quantizer would otherwise merge into the brown trunks).
    tiles = picked[::max(1, len(picked) // 8)][:8]
    w, h = picked[0].size
    mosaic = Image.new('RGB', (w * len(tiles), h + 40))
    for i, im in enumerate(tiles):
        mosaic.paste(im, (i * w, 0))
    for i, red in enumerate(((200, 30, 30), (150, 20, 20), (230, 60, 50), (110, 15, 15))):
        mosaic.paste(red, (i * 60, h, i * 60 + 60, h + 40))
    ref = mosaic.quantize(colors=128, method=Image.Quantize.MEDIANCUT)
    pal = [im.quantize(palette=ref, dither=Image.Dither.NONE) for im in picked]
    pal[0].save(out, save_all=True, append_images=pal[1:], duration=int(1000 / fps), loop=0,
                optimize=True)
    return len(pal)


def save_jpg(path, out, width=None, crop=None, text=None):
    img = Image.open(path)
    if crop:
        img = img.crop(crop)
    if width and img.width > width:
        img = img.resize((width, round(img.height * width / img.width)), Image.LANCZOS)
    if text:
        img = hud(img, text, max(12, img.width // 60))
    img.save(out, quality=85, optimize=True)


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument('--raw', default=os.path.join(root, 'artifacts/media_raw'))
    ap.add_argument('--out', default=os.path.join(root, 'docs/media'))
    ap.add_argument('--gif-start', type=float, default=0.0)
    ap.add_argument('--gif-end', type=float, default=240.0)
    ap.add_argument('--gif-step', type=float, default=2.0, help='sim seconds between GIF frames')
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--gif-width', type=int, default=640)
    ap.add_argument('--overview-t', type=float)
    ap.add_argument('--closeup-t', type=float)
    ap.add_argument('--depot-t', type=float)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    stats = load_stats(args.raw)

    n = make_gif(load_frames(args.raw, 'chase'), stats, os.path.join(args.out, 'demo.gif'),
                 args.gif_start, args.gif_end, args.gif_step, args.fps, args.gif_width)
    print(f'demo.gif: {n} frames')

    # Early in the mission: the robot at the nearest tree, apples still under every tree.
    t = args.overview_t if args.overview_t is not None else 30.0
    save_jpg(frame_at(load_frames(args.raw, 'overview'), t)[1],
             os.path.join(args.out, 'overview.jpg'), width=1200, crop=(150, 20, 1450, 750))
    t2 = args.closeup_t if args.closeup_t is not None else first_t(stats, 'PICK', after=30.0)
    save_jpg(frame_at(load_frames(args.raw, 'chase'), t2)[1], os.path.join(args.out, 'closeup.jpg'),
             width=1024)
    t3 = args.depot_t if args.depot_t is not None else first_t(stats, 'UNLOAD', last=True)
    save_jpg(frame_at(load_frames(args.raw, 'depot'), t3)[1], os.path.join(args.out, 'depot.jpg'),
             width=1024)
    print(f'overview t={t}, closeup t={t2}, depot t={t3}')
    for f in ('demo.gif', 'overview.jpg', 'closeup.jpg', 'depot.jpg'):
        print(f'{f}: {os.path.getsize(os.path.join(args.out, f)) / 1024:.0f} KB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
