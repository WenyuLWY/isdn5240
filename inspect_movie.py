"""Extract actual H.264 frames, at the encoder's original time marks, for visual inspection."""
from PIL import Image,ImageDraw,ImageFont
from pathlib import Path
import json,subprocess,io,sys
movie=Path(sys.argv[1]);record=json.loads(movie.with_suffix('.json').read_text())
keys=['selection','held','test_lift','dual','moving','release','push_clearance','wall'];sheet=Image.new('RGB',(1000,4*302),'#101e2d');font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
for j,key in enumerate(keys):
 a=record['video_markers'].get(key)
 if not a:continue
 proc=subprocess.check_output(['ffmpeg','-nostdin','-v','error','-ss',str(a['output_s']+.02),'-i',str(movie),'-vframes','1','-f','image2pipe','-vcodec','png','-'])
 im=Image.open(io.BytesIO(proc));im.thumbnail((496,274));x,y=(j%2)*500,(j//2)*302;sheet.paste(im,(x,y));ImageDraw.Draw(sheet).text((x+5,y+275),f"{key}    {a['output_s']:.2f} s in video",font=font,fill='white')
sheet.save(movie.with_name(movie.stem+'_quality.jpg'),quality=92)
