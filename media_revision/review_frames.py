import io
import json
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

video = Path(sys.argv[1])
info = json.loads(video.with_suffix('.json').read_text())
canvas = Image.new('RGB', (1200, 4*399), '#10202c')
draw = ImageDraw.Draw(canvas)
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 17)
marks = [(1, 'selection'), (1, 'lift'), (5, 'selection'), (5, 'updated'),
         (18, 'lift'), (29, 'selection'), (30, 'updated'), (30, 'end')]
for n, (slot, stage) in enumerate(marks):
    chapter = info['chapters'][slot-1]
    timestamp = info['duration_s'] - 1.2 if stage == 'end' else chapter[stage+'_video_s'] + .04
    raw = subprocess.check_output([
        'ffmpeg', '-nostdin', '-v', 'error', '-ss', f'{timestamp:.3f}', '-i', str(video),
        '-frames:v', '1', '-f', 'image2pipe', '-vcodec', 'png', '-',
    ])
    frame = Image.open(io.BytesIO(raw))
    frame.thumbnail((594, 370))
    x, y = (n % 2) * 600, (n // 2) * 399
    canvas.paste(frame, (x, y))
    draw.text((x+5, y+374), f"Brick {slot:02}  /  {stage}  /  {timestamp:.2f} s", font=font, fill='white')
canvas.save(video.with_name('review.jpg'), quality=92)
