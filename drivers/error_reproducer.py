import yaml
from utils.MPCC_solv_40N_no_warm_no_hard_invitedguest.interface import MPCC_solv_40N_no_warm_no_hard_invitedguest_py

def main():

    with open("param_dump.yaml", 'r') as dump:
        params = yaml.load(dump, Loader=yaml.Loader)

    MPCC_solv_40N_no_warm_no_hard_invitedguest_py.MPCC_solv_40N_no_warm_no_hard_invitedguest_solve(params)

    return None


if __name__=='__main__':
    main()