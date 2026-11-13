"""Prepare user-supplied paired JPEG references; no capture, model or robot operations."""
import argparse, datetime, hashlib, json, re
from pathlib import Path
from PIL import Image
from camera_ids import CAMERAS
ROOT = Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--set-id',required=True)
    p.add_argument('--objects-dir',type=Path,required=True)
    p.add_argument('--target-dir',type=Path,required=True)
    p.add_argument('--objects-captured-at',required=True)
    p.add_argument('--target-captured-at',required=True)
    a=p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',a.set_id):raise ValueError('Invalid set-id')
    if any(v.startswith('SET_') for v in CAMERAS.values()):raise ValueError('Configure cameras.local.json before preparing references')
    for stamp in (a.objects_captured_at,a.target_captured_at):datetime.datetime.fromisoformat(stamp)
    target=ROOT/'halo_skill_memory/task_references'/a.set_id
    if target.exists():raise FileExistsError('Reference set already exists; use a new set-id')
    sources=[]
    for key,role,folder,stamp in [('scene_01','initial_loose_blocks',a.objects_dir,a.objects_captured_at),('scene_02','assembled_building_target',a.target_dir,a.target_captured_at)]:
        rows=[]
        for camera,identity in CAMERAS.items():
            path=folder/(camera+'.jpg')
            with Image.open(path) as image:
                if image.format!='JPEG' or image.size!=(640,360):raise ValueError('Expected 640x360 JPEG: '+str(path))
                image.verify()
            raw=path.read_bytes()
            if not 100<=len(raw)<=3*1024*1024:raise ValueError('Reference JPEG outside size bounds')
            rows.append((camera,identity,raw))
        sources.append((key,role,stamp,rows))
    target.mkdir(parents=True)
    reference={'schema':'franka_task_reference_set_v1','reference_set_id':a.set_id}
    for key,role,stamp,rows in sources:
        folder=target/key;folder.mkdir();images=[]
        for camera,identity,raw in rows:
            image=folder/(camera+'.jpg');image.write_bytes(raw)
            images.append({'camera_id':camera,'camera_identity':identity,'width':640,'height':360,'path':str(image.relative_to(ROOT)),'sha256':hashlib.sha256(raw).hexdigest()})
        manifest=folder/'scene_manifest.json'
        manifest.write_text(json.dumps({'reference_set_id':a.set_id,'role':role,'captured_at':stamp,'images':images},ensure_ascii=False,indent=2)+'\n')
        reference[key]={'status':'saved','role':role,'manifest':str(manifest.relative_to(ROOT))}
    path=target/'reference_set.json';path.write_text(json.dumps(reference,ensure_ascii=False,indent=2)+'\n');print(path)
if __name__=='__main__':main()
