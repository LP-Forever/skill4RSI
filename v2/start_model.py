"""Start only the V2 model listener. No robot reset, gripper or motion commands."""
import argparse, sys
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='halo_skills.yaml')
    p.add_argument('--task-file',type=Path,required=True)
    p.add_argument('--reference-set',type=Path)
    p.add_argument('--port',type=int,default=7777)
    p.add_argument('--finish',choices=['hold','open_reset'],default='hold')
    p.add_argument('--no-record',action='store_true')
    a=p.parse_args()
    task=a.task_file.read_text().strip()
    if not task or '本文件是任务模板' in task:raise ValueError('Replace the example with an explicit task first')
    from halo_skill_service import main as serve
    sys.argv=['halo_skill_service.py','--config',a.config,'--task-prompt',task,'--port',str(a.port),'--finish',a.finish]
    if a.reference_set:sys.argv+=['--reference-set',str(a.reference_set)]
    if a.no_record:sys.argv+=['--no-record']
    serve()
if __name__=='__main__':main()
