"""Operator entry for the CURRENT V2 skill chain; importing never executes a task."""
from halo_pipeline_skills import main
if __name__ == '__main__':
    raise SystemExit(main())
