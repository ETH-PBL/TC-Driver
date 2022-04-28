import yaml
import osqp

with open("../param_dump.yaml", 'r') as dump:
    params = yaml.load(dump, yaml.Loader)

prog = osqp.OSQP()
prog.setup(params['P'], params['q'], params['A'], params['l'], params['u'], warm_start=True, check_termination = 25, verbose = True, polish=False, eps_rel=0.5e-3, eps_abs=0.5e-3)
prog.codegen('./emosqp/', parameters='matrices')
